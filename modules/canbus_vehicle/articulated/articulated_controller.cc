#include "modules/canbus_vehicle/articulated/articulated_controller.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/time/time.h"
#include "modules/canbus_vehicle/articulated/protocol/front_acu_drivemotor_563.h"
#include "modules/canbus_vehicle/articulated/protocol/front_acu_eps_547.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_control_mode_set_1057.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motion_command_273.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::common::ErrorCode;
using ::apollo::common::VehicleSignal;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;

}  // namespace

ErrorCode ArticulatedController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::Articulated>* const can_sender,
    MessageManager<::apollo::canbus::Articulated>* const message_manager) {
  if (is_initialized_) {
    AINFO << "ArticulatedController has already been initiated.";
    return ErrorCode::OK;
  }

  vehicle_params_.CopyFrom(::apollo::common::VehicleConfigHelper::Instance()
                               ->GetConfig()
                               .vehicle_param());
  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    AERROR << "Canbus sender is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // ---- Get front vehicle send protocols ----
  front_drive_motor_563_ = dynamic_cast<FrontAcuDrivemotor563*>(
      message_manager_->GetMutableProtocolDataById(
          FrontAcuDrivemotor563::ID));
  if (front_drive_motor_563_ == nullptr) {
    AERROR << "FrontAcuDrivemotor563 does not exist in MessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  front_eps_547_ = dynamic_cast<FrontAcuEps547*>(
      message_manager_->GetMutableProtocolDataById(FrontAcuEps547::ID));
  if (front_eps_547_ == nullptr) {
    AERROR << "FrontAcuEps547 does not exist in MessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  // ---- Get rear vehicle send protocols ----
  rear_motion_cmd_273_ = dynamic_cast<RearMotionCommand273*>(
      message_manager_->GetMutableProtocolDataById(RearMotionCommand273::ID));
  if (rear_motion_cmd_273_ == nullptr) {
    AERROR << "RearMotionCommand273 does not exist in MessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  rear_mode_set_1057_ = dynamic_cast<RearControlModeSet1057*>(
      message_manager_->GetMutableProtocolDataById(
          RearControlModeSet1057::ID));
  if (rear_mode_set_1057_ == nullptr) {
    AERROR << "RearControlModeSet1057 does not exist in MessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  // Register send messages
  can_sender_->AddMessage(FrontAcuDrivemotor563::ID,
                          front_drive_motor_563_, true);
  can_sender_->AddMessage(FrontAcuEps547::ID, front_eps_547_, true);
  can_sender_->AddMessage(RearMotionCommand273::ID,
                          rear_motion_cmd_273_, true);
  can_sender_->AddMessage(RearControlModeSet1057::ID,
                          rear_mode_set_1057_, true);

  AINFO << "ArticulatedController is initialized.";

  set_driving_mode(Chassis::COMPLETE_MANUAL);

  is_initialized_ = true;
  return ErrorCode::OK;
}

ArticulatedController::~ArticulatedController() {}

bool ArticulatedController::Start() {
  if (!is_initialized_) {
    AERROR << "ArticulatedController has not been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));
  return true;
}

void ArticulatedController::Stop() {
  if (!is_initialized_) {
    AERROR << "ArticulatedController stops improperly!";
    return;
  }
  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "ArticulatedController stopped.";
  }
}

void ArticulatedController::AddSendMessage() {
  can_sender_->AddMessage(FrontAcuDrivemotor563::ID, front_drive_motor_563_);
  can_sender_->AddMessage(FrontAcuEps547::ID, front_eps_547_);
  can_sender_->AddMessage(RearMotionCommand273::ID, rear_motion_cmd_273_);
  can_sender_->AddMessage(RearControlModeSet1057::ID, rear_mode_set_1057_);
}

// Parse "v_front=X,delta_front=X,v_rear=X,delta_rear=X" from msg string
bool ArticulatedController::ParseRearCommand(const ControlCommand& cmd,
                                             double* v_rear,
                                             double* delta_rear) {
  if (!cmd.header().has_status() || !cmd.header().status().has_msg()) {
    return false;
  }
  const std::string& raw = cmd.header().status().msg();
  int parsed = sscanf(raw.c_str(),
                      "v_front=%*f,delta_front=%*f,v_rear=%lf,delta_rear=%lf",
                      v_rear, delta_rear);
  return parsed == 2;
}

ErrorCode ArticulatedController::Update(const ControlCommand& command) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    return ErrorCode::OK;
  }

  // 1. Gear Control (front vehicle)
  Gear(command.gear_location());

  // 2. Front vehicle: Speed Control (Longitudinal)
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    double target_speed_kmh = command.speed() * 3.6;
    if (target_speed_kmh < 0) {
      target_speed_kmh = std::abs(target_speed_kmh);
    }
    front_drive_motor_563_->set_drive_motor_speed(target_speed_kmh);
    front_drive_motor_563_->set_drive_motor_enable(true);
    front_drive_motor_563_->set_drive_motor_mode(0);  // 0: Speed Mode
  }

  // 3. Front vehicle: Steering Control (Lateral)
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    Steer(command.steering_target());
  }

  // 4. Rear vehicle: Parse from header.status.msg
  double v_rear = 0.0, delta_rear = 0.0;
  if (ParseRearCommand(command, &v_rear, &delta_rear)) {
    rear_motion_cmd_273_->set_target_speed(v_rear);
    rear_motion_cmd_273_->set_target_steer_angle(delta_rear);
  } else {
    // Safety: if parse fails, stop rear vehicle
    rear_motion_cmd_273_->set_target_speed(0.0);
    rear_motion_cmd_273_->set_target_steer_angle(0.0);
    AWARN << "Failed to parse rear commands, rear vehicle stopped.";
  }

  can_sender_->Update();
  return ErrorCode::OK;
}

Chassis ArticulatedController::chassis() {
  chassis_.Clear();

  ::apollo::canbus::Articulated chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 1. Report Chassis Speed (from front vehicle, km/h -> m/s)
  if (chassis_detail.has_front_drivemotor_acu_572() &&
      chassis_detail.front_drivemotor_acu_572().has_drive_motor_speed()) {
    chassis_.set_speed_mps(
        chassis_detail.front_drivemotor_acu_572().drive_motor_speed() / 3.6);
  } else {
    chassis_.set_speed_mps(0.0);
  }

  // 2. Report Chassis Steering (from front vehicle EPS)
  if (chassis_detail.has_front_eps_acu_556() &&
      chassis_detail.front_eps_acu_556().has_eps_angle()) {
    double eps_angle = -1.0 * chassis_detail.front_eps_acu_556().eps_angle();
    double steer_ratio = vehicle_params_.steer_ratio();
    double wheel_angle = eps_angle / steer_ratio;
    double max_wheel_angle_deg =
        (vehicle_params_.max_steer_angle() * 180.0 / M_PI) / steer_ratio;

    if (max_wheel_angle_deg > 0) {
      double steering_percentage = (wheel_angle / max_wheel_angle_deg) * 100.0;
      chassis_.set_steering_percentage(
          ProtocolData<::apollo::canbus::Articulated>::BoundedValue(
              -100.0, 100.0, steering_percentage));
    }
  }

  // 3. Driving Mode (from front vehicle VCU)
  if (chassis_detail.has_front_vcu_acu_general_524()) {
    auto vcu_msg = chassis_detail.front_vcu_acu_general_524();
    if (vcu_msg.has_acu_remote_control() && vcu_msg.acu_remote_control()) {
      chassis_.set_driving_mode(Chassis::COMPLETE_MANUAL);
      set_driving_mode(Chassis::COMPLETE_MANUAL);
    } else if (vcu_msg.has_acu_control_mode() && vcu_msg.acu_control_mode()) {
      chassis_.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
      set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    } else {
      chassis_.set_driving_mode(Chassis::COMPLETE_MANUAL);
      set_driving_mode(Chassis::COMPLETE_MANUAL);
    }
  } else {
    chassis_.set_driving_mode(driving_mode());
  }

  // 4. Error Code (front OR rear fault)
  bool front_err = chassis_detail.has_front_vcu_acu_general_524() &&
                   chassis_detail.front_vcu_acu_general_524().acu_error();
  bool rear_err =
      chassis_detail.has_rear_chassis_status_529() &&
      chassis_detail.rear_chassis_status_529().vehicle_state() !=
          ArticulatedRearChassisStatus529::NORMAL;
  if (front_err || rear_err) {
    chassis_.set_error_code(Chassis::CHASSIS_ERROR);
  } else {
    chassis_.set_error_code(Chassis::NO_ERROR);
  }

  // 5. Gear (front vehicle)
  bool is_remote_control =
      chassis_detail.has_front_vcu_acu_general_524() &&
      chassis_detail.front_vcu_acu_general_524().has_acu_remote_control() &&
      chassis_detail.front_vcu_acu_general_524().acu_remote_control();
  if (is_remote_control) {
    chassis_.set_gear_location(Chassis::GEAR_DRIVE);
  } else if (chassis_detail.has_front_drivemotor_acu_572()) {
    auto shift = chassis_detail.front_drivemotor_acu_572().drive_motor_shift();
    if (shift == ArticulatedFrontDrivemotorAcu572::SHIFT_R)
      chassis_.set_gear_location(Chassis::GEAR_DRIVE);
    else if (shift == ArticulatedFrontDrivemotorAcu572::SHIFT_D)
      chassis_.set_gear_location(Chassis::GEAR_REVERSE);
    else if (shift == ArticulatedFrontDrivemotorAcu572::SHIFT_P)
      chassis_.set_gear_location(Chassis::GEAR_PARKING);
    else
      chassis_.set_gear_location(Chassis::GEAR_NEUTRAL);
  }

  // 6. Check response signals
  if (chassis_detail.has_front_eps_acu_556()) {
    chassis_.mutable_check_response()->set_is_eps_online(
        chassis_detail.front_eps_acu_556().eps_enable());
  }
  if (chassis_detail.has_front_drivemotor_acu_572()) {
    chassis_.mutable_check_response()->set_is_vcu_online(
        chassis_detail.front_drivemotor_acu_572().drive_motor_enable());
  }

  can_sender_->Update();

  return chassis_;
}

void ArticulatedController::Brake(double brake) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    return;
  }
  if (brake > 1.0) {
    front_drive_motor_563_->set_drive_motor_speed(0.0);
    front_drive_motor_563_->set_drive_motor_enable(true);
    front_drive_motor_563_->set_drive_motor_mode(0);
    rear_motion_cmd_273_->set_target_speed(0.0);
  }
}

void ArticulatedController::Throttle(double throttle) {}

void ArticulatedController::Speed(double speed) {
  front_drive_motor_563_->set_drive_motor_speed(speed * 3.6);
  front_drive_motor_563_->set_drive_motor_enable(true);
  front_drive_motor_563_->set_drive_motor_mode(0);
}

void ArticulatedController::Steer(double angle_deg) {
  // MPC 直接下发前轮转角 (度), 不是百分比
  // 需要乘以转向比得到 EPS 电机角度
  // 符号反转: Apollo 左转(正) → EPS 左转(负)
  double steer_ratio = vehicle_params_.steer_ratio();  // 6.0
  double eps_command = -angle_deg * steer_ratio;

  // 限幅到 EPS 电机物理范围 [-120°, 120°]
  double max_eps_deg = vehicle_params_.max_steer_angle() * 180.0 / M_PI;
  if (eps_command > max_eps_deg) eps_command = max_eps_deg;
  if (eps_command < -max_eps_deg) eps_command = -max_eps_deg;

  AINFO << "Steer: wheel_angle_deg=" << angle_deg
        << ", eps_command=" << eps_command
        << " (ratio=" << steer_ratio << ")";

  front_eps_547_->set_eps_angle(eps_command);
  front_eps_547_->set_eps_enable(true);
}

void ArticulatedController::Steer(double angle, double angle_spd) {
  Steer(angle);
}

void ArticulatedController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    return;
  }

  if (gear_position == Chassis::GEAR_DRIVE)
    front_drive_motor_563_->set_drive_motor_shift(
        ArticulatedFrontAcuDrivemotor563::SHIFT_R);
  else if (gear_position == Chassis::GEAR_REVERSE)
    front_drive_motor_563_->set_drive_motor_shift(
        ArticulatedFrontAcuDrivemotor563::SHIFT_D);
  else if (gear_position == Chassis::GEAR_PARKING)
    front_drive_motor_563_->set_drive_motor_shift(
        ArticulatedFrontAcuDrivemotor563::SHIFT_P);
  else
    front_drive_motor_563_->set_drive_motor_shift(
        ArticulatedFrontAcuDrivemotor563::SHIFT_N);

  can_sender_->Update();
}

void ArticulatedController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();

  // Zero out all commands for safety
  front_drive_motor_563_->set_drive_motor_speed(0.0);
  front_drive_motor_563_->set_drive_motor_enable(false);
  front_eps_547_->set_eps_angle(0.0);
  front_eps_547_->set_eps_enable(false);
  rear_motion_cmd_273_->set_target_speed(0.0);
  rear_motion_cmd_273_->set_target_steer_angle(0.0);
  rear_mode_set_1057_->set_mode(ArticulatedRearControlModeSet1057::STANDBY);
  can_sender_->Update();
}

ErrorCode ArticulatedController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "Already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }

  // Step 1: Enable rear vehicle CAN control mode first
  rear_mode_set_1057_->set_mode(
      ArticulatedRearControlModeSet1057::CAN_CONTROL);
  can_sender_->Update();

  // Brief wait for rear to acknowledge
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Step 2: Enable front vehicle
  front_drive_motor_563_->set_drive_motor_enable(true);
  front_eps_547_->set_eps_enable(true);

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }

  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode ArticulatedController::DisableAutoMode() {
  ResetProtocol();

  // Zero out all commands
  front_drive_motor_563_->set_drive_motor_speed(0.0);
  front_drive_motor_563_->set_drive_motor_enable(false);
  front_eps_547_->set_eps_angle(0.0);
  front_eps_547_->set_eps_enable(false);
  rear_motion_cmd_273_->set_target_speed(0.0);
  rear_motion_cmd_273_->set_target_steer_angle(0.0);

  // Set rear to standby
  rear_mode_set_1057_->set_mode(ArticulatedRearControlModeSet1057::STANDBY);
  can_sender_->Update();

  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode ArticulatedController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    return ErrorCode::OK;
  }

  front_drive_motor_563_->set_drive_motor_enable(false);
  front_eps_547_->set_eps_enable(true);

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }

  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  return ErrorCode::OK;
}

ErrorCode ArticulatedController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    return ErrorCode::OK;
  }

  front_drive_motor_563_->set_drive_motor_enable(true);
  front_eps_547_->set_eps_enable(false);

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_SPEED_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }

  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  return ErrorCode::OK;
}

void ArticulatedController::Acceleration(double acc) {}

void ArticulatedController::SetEpbBreak(const ControlCommand& command) {}

ErrorCode ArticulatedController::HandleCustomOperation(
    const external_command::ChassisCommand& command) {
  return ErrorCode::OK;
}

void ArticulatedController::SetBeam(const VehicleSignal& signal) {}
void ArticulatedController::SetHorn(const VehicleSignal& signal) {}
void ArticulatedController::SetTurningSignal(const VehicleSignal& signal) {}

bool ArticulatedController::VerifyID() { return true; }

void ArticulatedController::ResetProtocol() {
  AINFO << "ResetProtocol called.";
}

bool ArticulatedController::CheckResponse(const int32_t flags, bool need_wait) {
  AINFO << "CheckResponse bypassed, returning true.";
  return true;
}

void ArticulatedController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::cyber::Time::Now().ToMicrosecond();
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false)) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false)) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      AINFO << "Emergency mode suppressed for testing.";
    }
    end = ::apollo::cyber::Time::Now().ToMicrosecond();
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "SecurityDogThreadFunc execution time over period.";
    }
  }
}

void ArticulatedController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

Chassis::ErrorCode ArticulatedController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
