#pragma once

#include <mutex>

#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/common_msgs/basic_msgs/error_code.pb.h"
#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/external_command_msgs/chassis_command.pb.h"

#include "modules/canbus/vehicle/vehicle_controller.h"

namespace apollo {
namespace canbus {
namespace articulated {

class FrontAcuDrivemotor563;
class FrontAcuEps547;
class RearMotionCommand273;
class RearControlModeSet1057;
class RearErrorClearCommand1089;

class ArticulatedController final
    : public ::apollo::canbus::VehicleController<
          ::apollo::canbus::Articulated> {
 public:
  ArticulatedController() {}
  virtual ~ArticulatedController();

  ::apollo::common::ErrorCode Init(
      const VehicleParameter& params,
      ::apollo::drivers::canbus::CanSender<
          ::apollo::canbus::Articulated>* const can_sender,
      ::apollo::drivers::canbus::MessageManager<
          ::apollo::canbus::Articulated>* const message_manager) override;

  bool Start() override;
  void Stop() override;
  Chassis chassis() override;
  void AddSendMessage() override;

  // Override Update to handle articulated vehicle control
  common::ErrorCode Update(const control::ControlCommand& command) override;

  // Control Commands
  void Brake(double brake) override;
  void Throttle(double throttle) override;
  void Speed(double speed) override;
  void Steer(double steer_angle) override;
  void Gear(Chassis::GearPosition gear_position) override;

  void Emergency() override;
  common::ErrorCode EnableAutoMode() override;
  common::ErrorCode DisableAutoMode() override;
  common::ErrorCode EnableSteeringOnlyMode() override;
  common::ErrorCode EnableSpeedOnlyMode() override;
  void Acceleration(double acc) override;
  void Steer(double angle, double angle_spd) override;
  void SetEpbBreak(const control::ControlCommand& command) override;
  common::ErrorCode HandleCustomOperation(
      const external_command::ChassisCommand& command) override;
  void SetBeam(const common::VehicleSignal& signal) override;
  void SetHorn(const common::VehicleSignal& signal) override;
  void SetTurningSignal(const common::VehicleSignal& signal) override;
  bool VerifyID() override;
  bool CheckResponse(const int32_t flags, bool need_wait);
  void SecurityDogThreadFunc();

 private:
  void ResetProtocol();
  void set_chassis_error_code(const Chassis::ErrorCode& error_code);
  Chassis::ErrorCode chassis_error_code();

  // Parse rear vehicle commands from ControlCommand.header.status.msg
  bool ParseRearCommand(const control::ControlCommand& cmd,
                        double* v_rear, double* delta_rear);

 private:
  Chassis chassis_;
  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::unique_ptr<std::thread> thread_;

  // Front vehicle control protocols
  FrontAcuDrivemotor563* front_drive_motor_563_ = nullptr;
  FrontAcuEps547* front_eps_547_ = nullptr;

  // Rear vehicle control protocols
  RearMotionCommand273* rear_motion_cmd_273_ = nullptr;
  RearControlModeSet1057* rear_mode_set_1057_ = nullptr;
  RearErrorClearCommand1089* rear_error_clear_1089_ = nullptr;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
