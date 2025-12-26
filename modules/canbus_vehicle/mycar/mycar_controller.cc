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
  can_sender_->AddMessage(AcuDrivemotor563::ID, drive_motor_563_);
  can_sender_->AddMessage(AcuEps547::ID, eps_547_);
}

ErrorCode MycarController::Update(const ControlCommand& command) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    return ErrorCode::OK;
  }

  // 1. Gear Control
  Gear(command.gear_location());

  // 2. Speed Control (Longitudinal)
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    // Map m/s to km/h
    double target_speed_kmh = command.speed() * 3.6;

    // Ensure speed is positive magnitude (direction handled by Gear)
    if (target_speed_kmh < 0) {
      target_speed_kmh = std::abs(target_speed_kmh);
    }

    drive_motor_563_->set_drive_motor_speed(target_speed_kmh);
    drive_motor_563_->set_drive_motor_enable(true);
    drive_motor_563_->set_drive_motor_mode(0);  // 0: Speed Mode
  }

  // 3. Steering Control (Lateral)
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    Steer(command.steering_target());
  }

  can_sender_->Update();
  return ErrorCode::OK;
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
  // EPS反馈: 左转为负(-0 to -120), 右转为正(0 to 120) - 这是方向盘/电机角度
  // Apollo约定: 左转为正, 右转为负 - 需要的是车轮角度
  // 转换步骤:
  //   1. 符号反转: EPS左转(负) -> Apollo左转(正)
  //   2. 转向比换算: 方向盘角度 / steer_ratio = 车轮角度
  if (chassis_detail.has_eps_acu_556() &&
      chassis_detail.eps_acu_556().has_eps_angle()) {
    // Step 1: 符号反转 - 车辆左转(负) -> Apollo左转(正)
    double eps_angle = -1.0 * chassis_detail.eps_acu_556().eps_angle();

    // Step 2: 转向比换算 - 方向盘角度转车轮角度
    // steer_ratio = 6.0, 即方向盘转6度 = 车轮转1度
    // 车轮角度 = 方向盘角度 / steer_ratio
    double steer_ratio = vehicle_params_.steer_ratio();
    double wheel_angle = eps_angle / steer_ratio;

    // 计算百分比: 车轮角度范围是 ±(max_steer_angle / steer_ratio)
    // max_steer_angle = 2.0944 rad (120°), 对应车轮最大角度 = 120° / 6 = 20°
    double max_wheel_angle_deg =
        (vehicle_params_.max_steer_angle() * 180.0 / M_PI) / steer_ratio;

    if (max_wheel_angle_deg > 0) {
      // 百分比 = (当前车轮角度 / 最大车轮角度) * 100
      double steering_percentage = (wheel_angle / max_wheel_angle_deg) * 100.0;
      chassis_.set_steering_percentage(
          ::apollo::drivers::canbus::ProtocolData<
              ::apollo::canbus::Mycar>::BoundedValue(-100.0, 100.0,
                                                     steering_percentage));

      AINFO << "Steering feedback: eps_angle=" << eps_angle
            << ", wheel_angle=" << wheel_angle
            << ", percentage=" << steering_percentage;
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

  // 5. Gear - 车辆CAN值: SHIFT_D=1(车辆后退), SHIFT_R=2(车辆前进)
  //         Apollo: GEAR_DRIVE=1(前进), GEAR_REVERSE=2(后退)
  //         遥控模式下档位不可靠，默认使用GEAR_DRIVE
  bool is_remote_control =
      chassis_detail.has_vcu_acu_general_524() &&
      chassis_detail.vcu_acu_general_524().has_acu_remote_control() &&
      chassis_detail.vcu_acu_general_524().acu_remote_control();
  if (is_remote_control) {
    // 遥控模式：档位信息不可靠，默认为前进档
    chassis_.set_gear_location(Chassis::GEAR_DRIVE);
  } else if (chassis_detail.has_drivemotor_acu_572()) {
    // 自动驾驶模式：按CAN消息解析真实档位
    auto shift = chassis_detail.drivemotor_acu_572().drive_motor_shift();
    if (shift == ::apollo::canbus::MycarDrivemotorAcu572::SHIFT_R)
      chassis_.set_gear_location(
          Chassis::GEAR_DRIVE);  // SHIFT_R(值2,车辆前进) → GEAR_DRIVE
    else if (shift == ::apollo::canbus::MycarDrivemotorAcu572::SHIFT_D)
      chassis_.set_gear_location(
          Chassis::GEAR_REVERSE);  // SHIFT_D(值1,车辆后退) → GEAR_REVERSE
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
  // Logic removed to prevent conflict with Update()
  // Speed is now controlled directly via Update() -> set_drive_motor_speed
  // AINFO << "MycarController::Throttle called (Ignored): " << throttle;
}

void MycarController::Speed(double speed) {
  AINFO << "MycarController::Speed called with: " << speed;
  drive_motor_563_->set_drive_motor_speed(speed * 3.6);  // Convert m/s to km/h
  drive_motor_563_->set_drive_motor_enable(true);
  drive_motor_563_->set_drive_motor_mode(0);  // 0 for Speed Mode
}

void MycarController::Steer(double angle) {
  // Apollo输入: 百分比 [-100, 100], 左转为正, 右转为负
  // EPS需要: 角度 [-120, 120], 左转为负, 右转为正
  // 因此需要符号反转

  // max_steer_angle 是EPS最大角度的弧度值 (120° = 2.0944 rad)
  const double max_angle_rad = vehicle_params_.max_steer_angle();

  // Step 1: 百分比 -> 弧度
  // target_rad = (percentage / 100.0) * max_angle_rad
  double target_rad = (angle / 100.0) * max_angle_rad;

  // Step 2: 弧度 -> 角度
  double target_deg = target_rad * 180.0 / M_PI;

  // Step 3: 符号反转 - Apollo左转(正) -> EPS左转(负)
  double eps_command = -1.0 * target_deg;

  AINFO << "Steer: percentage=" << angle << ", target_rad=" << target_rad
        << ", target_deg=" << target_deg << ", eps_command=" << eps_command;

  eps_547_->set_eps_angle(eps_command);
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

  // Apollo: GEAR_DRIVE(前进), GEAR_REVERSE(后退)
  // 车辆CAN: SHIFT_R=2(车辆前进), SHIFT_D=1(车辆后退)
  if (gear_position == Chassis::GEAR_DRIVE)
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::MycarAcuDrivemotor563::
            SHIFT_R);  // 前进 → SHIFT_R(值2,车辆前进)
  else if (gear_position == Chassis::GEAR_REVERSE)
    drive_motor_563_->set_drive_motor_shift(
        ::apollo::canbus::MycarAcuDrivemotor563::
            SHIFT_D);  // 后退 → SHIFT_D(值1,车辆后退)
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
