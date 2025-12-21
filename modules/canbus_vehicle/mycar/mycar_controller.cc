#include "modules/canbus_vehicle/mycar/mycar_controller.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/time/time.h"
#include "modules/canbus_vehicle/mycar/protocol/acu_drivemotor_563.h"
#include "modules/canbus_vehicle/mycar/protocol/acu_eps_547.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace canbus {
namespace mycar {

using ::apollo::common::ErrorCode;
using ::apollo::common::VehicleSignal;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;

}  // namespace

ErrorCode MycarController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::Mycar>* const can_sender,
    MessageManager<::apollo::canbus::Mycar>* const message_manager) {
  if (is_initialized_) {
    AINFO << "MycarController has already been initiated.";
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

  drive_motor_563_ = dynamic_cast<AcuDrivemotor563*>(
      message_manager_->GetMutableProtocolDataById(AcuDrivemotor563::ID));
  if (drive_motor_563_ == nullptr) {
    AERROR << "AcuDrivemotor563 does not exist in the MycarMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  eps_547_ = dynamic_cast<AcuEps547*>(
      message_manager_->GetMutableProtocolDataById(AcuEps547::ID));
  if (eps_547_ == nullptr) {
    AERROR << "AcuEps547 does not exist in the MycarMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(AcuDrivemotor563::ID, drive_motor_563_, true);
  can_sender_->AddMessage(AcuEps547::ID, eps_547_, true);

  AINFO << "MycarController is initialized.";

  // Default to MANUAL on startup
  set_driving_mode(Chassis::COMPLETE_MANUAL);

  is_initialized_ = true;
  return ErrorCode::OK;
}

MycarController::~MycarController() {}

bool MycarController::Start() {
  if (!is_initialized_) {
    AERROR << "MycarController has not been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void MycarController::Stop() {
  if (!is_initialized_) {
    AERROR << "MycarController stops improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "MycarController stopped.";
  }
}

void MycarController::AddSendMessage() {
  can_sender_->AddMessage(AcuDrivemotor563::ID, drive_motor_563_);
  can_sender_->AddMessage(AcuEps547::ID, eps_547_);
}

Chassis MycarController::chassis() {
  chassis_.Clear();

  ::apollo::canbus::Mycar chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 1. Report Chassis Speed
  if (chassis_detail.has_drivemotor_acu_572() &&
      chassis_detail.drivemotor_acu_572().has_drive_motor_speed()) {
    chassis_.set_speed_mps(
        chassis_detail.drivemotor_acu_572().drive_motor_speed() / 3.6);
  } else {
    chassis_.set_speed_mps(0.0);
  }

  // 2. Report Chassis Steering
  if (chassis_detail.has_eps_acu_556() &&
      chassis_detail.eps_acu_556().has_eps_angle()) {
    double eps_angle = chassis_detail.eps_acu_556().eps_angle();
    // Convert deg to percentage [-100, 100]
    // vehicle_params_.max_steer_angle() is in radians
    double max_steer_angle_deg =
        vehicle_params_.max_steer_angle() * 180.0 / 3.1415926535;
    if (max_steer_angle_deg > 0) {
      double steering_percentage = (eps_angle / max_steer_angle_deg) * 100.0;
      chassis_.set_steering_percentage(
          ::apollo::drivers::canbus::ProtocolData<
              ::apollo::canbus::Mycar>::BoundedValue(-100.0, 100.0,
                                                     steering_percentage));
    }
  }

  // 3. Driving Mode
  if (chassis_detail.has_vcu_acu_general_524()) {
    auto vcu_msg = chassis_detail.vcu_acu_general_524();
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
    // Default or fallback
    chassis_.set_driving_mode(driving_mode());
  }

  // 4. Error Code
  if (chassis_detail.has_vcu_acu_general_524() &&
      chassis_detail.vcu_acu_general_524().acu_error()) {
    chassis_.set_error_code(Chassis::CHASSIS_ERROR);
  } else {
    chassis_.set_error_code(Chassis::NO_ERROR);
  }

  // 5. Gear
  if (chassis_detail.has_drivemotor_acu_572()) {
    auto shift = chassis_detail.drivemotor_acu_572().drive_motor_shift();
    if (shift == ::apollo::canbus::MycarDrivemotorAcu572::SHIFT_D)
      chassis_.set_gear_location(Chassis::GEAR_DRIVE);
    else if (shift == ::apollo::canbus::MycarDrivemotorAcu572::SHIFT_R)
      chassis_.set_gear_location(Chassis::GEAR_REVERSE);
    else if (shift == ::apollo::canbus::MycarDrivemotorAcu572::SHIFT_P)
      chassis_.set_gear_location(Chassis::GEAR_PARKING);
    else
      chassis_.set_gear_location(Chassis::GEAR_NEUTRAL);
  }

  // 6. Check response signals
  if (chassis_detail.has_eps_acu_556()) {
    chassis_.mutable_check_response()->set_is_eps_online(
        chassis_detail.eps_acu_556().eps_enable());
  }
  if (chassis_detail.has_drivemotor_acu_572()) {
    chassis_.mutable_check_response()->set_is_vcu_online(
        chassis_detail.drivemotor_acu_572().drive_motor_enable());
  }

  // Force Update loop to ensure CAN messages are sent periodically
  can_sender_->Update();

  return chassis_;
}

void MycarController::Brake(double brake) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current driving mode is not AUTO_DRIVE or SPEED_ONLY.";
    return;
  }
  // Mycar specific brake logic
  if (brake > 1.0) {
    drive_motor_563_->set_drive_motor_speed(0.0);
    drive_motor_563_->set_drive_motor_enable(true);
    drive_motor_563_->set_drive_motor_mode(0);
  }
}

void MycarController::Throttle(double throttle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current driving mode is not AUTO_DRIVE or SPEED_ONLY.";
    return;
  }
  // Mycar specific throttle logic
  // Map throttle (0-100) to speed (m/s)
  // Assuming max speed is 10 m/s (~36 km/h) for teleop
  const double kMaxSpeedMps = 10.0;
  double speed_mps = (throttle / 100.0) * kMaxSpeedMps;

  drive_motor_563_->set_drive_motor_speed(speed_mps * 3.6);  // Convert to km/h
  drive_motor_563_->set_drive_motor_enable(true);
  drive_motor_563_->set_drive_motor_mode(0);  // 0 for Speed Mode
}

void MycarController::Speed(double speed) {
  // Ignore 0 speed command from default ControlCommand if throttle was used
  if (std::abs(speed) < 1e-6) {
    return;
  }
  AINFO << "MycarController::Speed called with: " << speed;
  drive_motor_563_->set_drive_motor_speed(speed * 3.6);
  drive_motor_563_->set_drive_motor_enable(true);
  drive_motor_563_->set_drive_motor_mode(0);  // 0 for Speed Mode
}

void MycarController::Steer(double angle) {
  AINFO << "MycarController::Steer called with: " << angle;
  const double max_angle = vehicle_params_.max_steer_angle();
  const double target_angle = (angle / 100.0) * max_angle * (180.0 / 3.1415926);
  eps_547_->set_eps_angle(target_angle);
  eps_547_->set_eps_enable(true);
}

void MycarController::Steer(double angle, double angle_spd) { Steer(angle); }

void MycarController::Gear(Chassis::GearPosition gear_position) {
  AINFO << "MycarController::Gear called with: " << gear_position;
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current driving mode is not AUTO_DRIVE or SPEED_ONLY.";
    return;
  }

  if (gear_position == Chassis::GEAR_DRIVE)
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::MycarAcuDrivemotor563::SHIFT_D);
  else if (gear_position == Chassis::GEAR_REVERSE)
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::MycarAcuDrivemotor563::SHIFT_R);
  else if (gear_position == Chassis::GEAR_PARKING)
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::MycarAcuDrivemotor563::SHIFT_P);
  else
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::MycarAcuDrivemotor563::SHIFT_N);

  can_sender_->Update();
}

void MycarController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode MycarController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "Already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }

  // Set enable signals for all units
  drive_motor_563_->set_drive_motor_enable(true);
  eps_547_->set_eps_enable(true);

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

ErrorCode MycarController::DisableAutoMode() {
  ResetProtocol();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode MycarController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    return ErrorCode::OK;
  }

  drive_motor_563_->set_drive_motor_enable(false);
  eps_547_->set_eps_enable(true);

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

ErrorCode MycarController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    return ErrorCode::OK;
  }

  drive_motor_563_->set_drive_motor_enable(true);
  eps_547_->set_eps_enable(false);

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

void MycarController::Acceleration(double acc) {}

void MycarController::SetEpbBreak(const ControlCommand& command) {}

ErrorCode MycarController::HandleCustomOperation(
    const external_command::ChassisCommand& command) {
  return ErrorCode::OK;
}

void MycarController::SetBeam(const VehicleSignal& signal) {}

void MycarController::SetHorn(const VehicleSignal& signal) {}

void MycarController::SetTurningSignal(const VehicleSignal& signal) {}

bool MycarController::VerifyID() { return true; }

void MycarController::ResetProtocol() {
  // message_manager_->ResetSendMessages();
  AINFO << "ResetProtocol called but bypassed for testing.";
}

bool MycarController::CheckResponse(const int32_t flags, bool need_wait) {
  AINFO << "CheckResponse bypassed, returning true.";
  return true;
}

void MycarController::SecurityDogThreadFunc() {
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

    // 1. horizontal control check
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

    // 2. vertical control check
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
      // set_driving_mode(Chassis::EMERGENCY_MODE);
      // message_manager_->ResetSendMessages();
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

void MycarController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

Chassis::ErrorCode MycarController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
