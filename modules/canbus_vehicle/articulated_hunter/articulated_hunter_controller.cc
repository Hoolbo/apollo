#include "modules/canbus_vehicle/articulated_hunter/articulated_hunter_controller.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/time/time.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

using ::apollo::common::ErrorCode;
using ::apollo::common::VehicleSignal;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;

}  // namespace

ArticulatedHunterController::ArticulatedHunterController() {}

ArticulatedHunterController::~ArticulatedHunterController() {}

ErrorCode ArticulatedHunterController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ArticulatedHunter>* const can_sender,
    MessageManager<::apollo::canbus::ArticulatedHunter>* const message_manager) {
  if (is_initialized_) {
    AINFO << "ArticulatedHunterController has already been initiated.";
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

  // ====== 初始化主车(Mycar) 控制协议 ======
  drive_motor_563_ = dynamic_cast<AcuDrivemotor563*>(
      message_manager_->GetMutableProtocolDataById(AcuDrivemotor563::ID));
  if (drive_motor_563_ == nullptr) {
    AERROR << "AcuDrivemotor563 does not exist in the ArticulatedHunterMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  eps_547_ = dynamic_cast<AcuEps547*>(
      message_manager_->GetMutableProtocolDataById(AcuEps547::ID));
  if (eps_547_ == nullptr) {
    AERROR << "AcuEps547 does not exist in the ArticulatedHunterMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  // ====== 初始化从车(Hunter) 控制协议 ======
  control_mode_set_421_ = dynamic_cast<Controlmodeset421*>(
      message_manager_->GetMutableProtocolDataById(Controlmodeset421::ID));
  if (control_mode_set_421_ == nullptr) {
    AERROR << "Controlmodeset421 does not exist in the ArticulatedHunterMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  error_clear_command_441_ = dynamic_cast<Errorclearcommand441*>(
      message_manager_->GetMutableProtocolDataById(Errorclearcommand441::ID));
  if (error_clear_command_441_ == nullptr) {
    AERROR << "Errorclearcommand441 does not exist in the ArticulatedHunterMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  motion_command_111_ = dynamic_cast<Motioncommand111*>(
      message_manager_->GetMutableProtocolDataById(Motioncommand111::ID));
  if (motion_command_111_ == nullptr) {
    AERROR << "Motioncommand111 does not exist in the ArticulatedHunterMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  // 注册所有要发送的消息
  AddSendMessage();

  AINFO << "ArticulatedHunterController is initialized.";

  set_driving_mode(Chassis::COMPLETE_MANUAL);

  is_initialized_ = true;
  return ErrorCode::OK;
}

bool ArticulatedHunterController::Start() {
  if (!is_initialized_) {
    AERROR << "ArticulatedHunterController has not been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void ArticulatedHunterController::Stop() {
  if (!is_initialized_) {
    AERROR << "ArticulatedHunterController stops improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "ArticulatedHunterController stopped.";
  }
}

void ArticulatedHunterController::AddSendMessage() {
  // ====== 主车(Mycar) 消息 ======
  can_sender_->AddMessage(AcuDrivemotor563::ID, drive_motor_563_, true);
  can_sender_->AddMessage(AcuEps547::ID, eps_547_, true);

  // ====== 从车(Hunter) 消息 ======
  can_sender_->AddMessage(Controlmodeset421::ID, control_mode_set_421_, false);
  can_sender_->AddMessage(Errorclearcommand441::ID, error_clear_command_441_, false);
  can_sender_->AddMessage(Motioncommand111::ID, motion_command_111_, false);
}

ErrorCode ArticulatedHunterController::Update(const ControlCommand& command) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    return ErrorCode::OK;
  }

  // 1. Gear Control (主车)
  Gear(command.gear_location());

  // 2. Speed Control (主车+从车 同时控制)
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    // 主车速度控制 (km/h)
    double target_speed_kmh = command.speed() * 3.6;
    if (target_speed_kmh < 0) {
      target_speed_kmh = std::abs(target_speed_kmh);
    }
    drive_motor_563_->set_drive_motor_speed(target_speed_kmh);
    drive_motor_563_->set_drive_motor_enable(true);
    drive_motor_563_->set_drive_motor_mode(0);  // 0: Speed Mode

    // 从车速度控制 (m/s)
    motion_command_111_->set_target_speed(command.speed());
  }

  // 3. Steering Control (主车+从车 同时控制)
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    // 主车转向
    Steer(command.steering_target());
    
    // 从车转向 - 同样的转角指令
    double max_steer_angle = vehicle_params_.max_steer_angle();
    double angle_hunter = (command.steering_target() / 100.0) * max_steer_angle;
    motion_command_111_->set_target_steer_angle(angle_hunter);
  }

  can_sender_->Update();
  return ErrorCode::OK;
}

Chassis ArticulatedHunterController::chassis() {
  chassis_.Clear();

  ::apollo::canbus::ArticulatedHunter chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // ====== 主车速度反馈 ======
  if (chassis_detail.has_drivemotor_acu_572() &&
      chassis_detail.drivemotor_acu_572().has_drive_motor_speed()) {
    chassis_.set_speed_mps(
        chassis_detail.drivemotor_acu_572().drive_motor_speed() / 3.6);
  } else {
    chassis_.set_speed_mps(0.0);
  }

  // ====== 主车转向反馈 ======
  if (chassis_detail.has_eps_acu_556() &&
      chassis_detail.eps_acu_556().has_eps_angle()) {
    double eps_angle = -1.0 * chassis_detail.eps_acu_556().eps_angle();
    double steer_ratio = vehicle_params_.steer_ratio();
    double wheel_angle = eps_angle / steer_ratio;
    double max_wheel_angle_deg =
        (vehicle_params_.max_steer_angle() * 180.0 / M_PI) / steer_ratio;

    if (max_wheel_angle_deg > 0) {
      double steering_percentage = (wheel_angle / max_wheel_angle_deg) * 100.0;
      chassis_.set_steering_percentage(
          ::apollo::drivers::canbus::ProtocolData<
              ::apollo::canbus::ArticulatedHunter>::BoundedValue(-100.0, 100.0,
                                                     steering_percentage));
    }
  } else if (chassis_detail.has_motion_feedback_221()) {
    // ====== 从车转向反馈(备选) ======
    Motion_feedback_221 motion_feedback_221 = chassis_detail.motion_feedback_221();
    if (motion_feedback_221.has_steering_angle()) {
      double steer_target = motion_feedback_221.steering_angle() * 100.0 / vehicle_params_.max_steer_angle();
      chassis_.set_steering_percentage(steer_target);
    }
  }

  // ====== 驾驶模式 ======
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
    chassis_.set_driving_mode(driving_mode());
  }

  // ====== 错误码 ======
  if (chassis_detail.has_vcu_acu_general_524() &&
      chassis_detail.vcu_acu_general_524().acu_error()) {
    chassis_.set_error_code(Chassis::CHASSIS_ERROR);
  } else {
    chassis_.set_error_code(Chassis::NO_ERROR);
  }

  // ====== 档位 ======
  bool is_remote_control =
      chassis_detail.has_vcu_acu_general_524() &&
      chassis_detail.vcu_acu_general_524().has_acu_remote_control() &&
      chassis_detail.vcu_acu_general_524().acu_remote_control();
  if (is_remote_control) {
    chassis_.set_gear_location(Chassis::GEAR_DRIVE);
  } else if (chassis_detail.has_drivemotor_acu_572()) {
    auto shift = chassis_detail.drivemotor_acu_572().drive_motor_shift();
    if (shift == ::apollo::canbus::ArticulatedHunterDrivemotorAcu572::SHIFT_R)
      chassis_.set_gear_location(Chassis::GEAR_DRIVE);
    else if (shift == ::apollo::canbus::ArticulatedHunterDrivemotorAcu572::SHIFT_D)
      chassis_.set_gear_location(Chassis::GEAR_REVERSE);
    else if (shift == ::apollo::canbus::ArticulatedHunterDrivemotorAcu572::SHIFT_P)
      chassis_.set_gear_location(Chassis::GEAR_PARKING);
    else
      chassis_.set_gear_location(Chassis::GEAR_NEUTRAL);
  }

  // ====== 检查响应信号 ======
  if (chassis_detail.has_eps_acu_556()) {
    chassis_.mutable_check_response()->set_is_eps_online(
        chassis_detail.eps_acu_556().eps_enable());
  }
  if (chassis_detail.has_drivemotor_acu_572()) {
    chassis_.mutable_check_response()->set_is_vcu_online(
        chassis_detail.drivemotor_acu_572().drive_motor_enable());
  }
  if (chassis_detail.has_chassis_status_211()) {
    auto chassis_status = chassis_detail.chassis_status_211();
    if (chassis_status.vehicle_state() == 
        ArticulatedHunterChassisStatus211_Vehicle_stateType_VEHICLE_STATE_NORMAL) {
      chassis_.mutable_check_response()->set_is_esp_online(true);
    }
  }

  can_sender_->Update();

  return chassis_;
}

void ArticulatedHunterController::Brake(double brake) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current driving mode is not AUTO_DRIVE or SPEED_ONLY.";
    return;
  }
  if (brake > 1.0) {
    drive_motor_563_->set_drive_motor_speed(0.0);
    drive_motor_563_->set_drive_motor_enable(true);
    drive_motor_563_->set_drive_motor_mode(0);
  }
}

void ArticulatedHunterController::Throttle(double throttle) {
  // Logic removed to prevent conflict with Update()
}

void ArticulatedHunterController::Speed(double speed) {
  AINFO << "ArticulatedHunterController::Speed called with: " << speed;
  drive_motor_563_->set_drive_motor_speed(speed * 3.6);
  drive_motor_563_->set_drive_motor_enable(true);
  drive_motor_563_->set_drive_motor_mode(0);
  motion_command_111_->set_target_speed(speed);
}

void ArticulatedHunterController::Steer(double angle) {
  const double max_angle_rad = vehicle_params_.max_steer_angle();
  double target_rad = (angle / 100.0) * max_angle_rad;
  double target_deg = target_rad * 180.0 / M_PI;
  double eps_command = -1.0 * target_deg;

  AINFO << "Steer: percentage=" << angle << ", target_rad=" << target_rad
        << ", target_deg=" << target_deg << ", eps_command=" << eps_command;

  eps_547_->set_eps_angle(eps_command);
  eps_547_->set_eps_enable(true);
}

void ArticulatedHunterController::Steer(double angle, double angle_spd) {
  Steer(angle);
}

void ArticulatedHunterController::Gear(Chassis::GearPosition gear_position) {
  AINFO << "ArticulatedHunterController::Gear called with: " << gear_position;
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current driving mode is not AUTO_DRIVE or SPEED_ONLY.";
    return;
  }

  if (gear_position == Chassis::GEAR_DRIVE)
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::ArticulatedHunterAcuDrivemotor563::SHIFT_R);
  else if (gear_position == Chassis::GEAR_REVERSE)
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::ArticulatedHunterAcuDrivemotor563::SHIFT_D);
  else if (gear_position == Chassis::GEAR_PARKING)
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::ArticulatedHunterAcuDrivemotor563::SHIFT_P);
  else
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::ArticulatedHunterAcuDrivemotor563::SHIFT_N);

  can_sender_->Update();
}

void ArticulatedHunterController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode ArticulatedHunterController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "Already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }

  // 主车使能
  drive_motor_563_->set_drive_motor_enable(true);
  eps_547_->set_eps_enable(true);

  // 从车使能
  control_mode_set_421_->set_mode_set(
      ArticulatedHunterControlModeSet421_Mode_setType_MODE_SET_CANCONTROL);

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

ErrorCode ArticulatedHunterController::DisableAutoMode() {
  ResetProtocol();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode ArticulatedHunterController::EnableSteeringOnlyMode() {
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

ErrorCode ArticulatedHunterController::EnableSpeedOnlyMode() {
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

void ArticulatedHunterController::Acceleration(double acc) {}

void ArticulatedHunterController::SetEpbBreak(const ControlCommand& command) {}

ErrorCode ArticulatedHunterController::HandleCustomOperation(
    const external_command::ChassisCommand& command) {
  return ErrorCode::OK;
}

void ArticulatedHunterController::SetBeam(const VehicleSignal& signal) {}

void ArticulatedHunterController::SetHorn(const VehicleSignal& signal) {}

void ArticulatedHunterController::SetTurningSignal(const VehicleSignal& signal) {}

bool ArticulatedHunterController::VerifyID() { return true; }

void ArticulatedHunterController::ResetProtocol() {
  AINFO << "ResetProtocol called.";
}

bool ArticulatedHunterController::CheckResponse(const int32_t flags, bool need_wait) {
  AINFO << "CheckResponse bypassed, returning true.";
  return true;
}

bool ArticulatedHunterController::CheckChassisError() {
  return false;
}

void ArticulatedHunterController::SecurityDogThreadFunc() {
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
      AINFO << "Emergency mode triggered.";
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

void ArticulatedHunterController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

Chassis::ErrorCode ArticulatedHunterController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void ArticulatedHunterController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t ArticulatedHunterController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
