#pragma once

#include <mutex>

#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus_vehicle/mycar/proto/mycar.pb.h"
#include "modules/common_msgs/basic_msgs/error_code.pb.h"
#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/external_command_msgs/chassis_command.pb.h"

#include "modules/canbus/vehicle/vehicle_controller.h"

namespace apollo {
namespace canbus {
namespace mycar {

class AcuDrivemotor563;
class AcuEps547;

class MycarController final
    : public ::apollo::canbus::VehicleController<::apollo::canbus::Mycar> {
 public:
  MycarController() {}
  virtual ~MycarController();

  ::apollo::common::ErrorCode Init(
      const VehicleParameter& params,
      ::apollo::drivers::canbus::CanSender<::apollo::canbus::Mycar>* const
          can_sender,
      ::apollo::drivers::canbus::MessageManager<::apollo::canbus::Mycar>* const
          message_manager) override;

  bool Start() override;
  void Stop() override;
  Chassis chassis() override;
  void AddSendMessage() override;

  // Control Commands
  void Brake(double brake) override;
  void Throttle(double throttle) override;
  void Speed(double speed) override;
  void Steer(double steer_angle) override;
  void Gear(Chassis::GearPosition gear_position) override;

  // New additions
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

 private:
  Chassis chassis_;
  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::unique_ptr<std::thread> thread_;

  // control protocol
  AcuDrivemotor563* drive_motor_563_ = nullptr;
  AcuEps547* eps_547_ = nullptr;
};

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
