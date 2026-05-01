#pragma once

#include <memory>
#include <thread>

#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus_vehicle/articulated_hunter/proto/articulated_hunter.pb.h"

// Mycar protocol headers
#include "modules/canbus_vehicle/articulated_hunter/protocol/acu_drivemotor_563.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/acu_eps_547.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/drivemotor_acu_572.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/eps_acu_556.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/vcu_acu_general_524.h"

// Hunter protocol headers
#include "modules/canbus_vehicle/articulated_hunter/protocol/chassis_status_211.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/control_mode_set_421.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/error_clear_command_441.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motion_command_111.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motion_feedback_221.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

class ArticulatedHunterController
    : public ::apollo::canbus::VehicleController<
          ::apollo::canbus::ArticulatedHunter> {
 public:
  ArticulatedHunterController();
  virtual ~ArticulatedHunterController();

  // VehicleController interface
  ErrorCode Init(
      const VehicleParameter& params,
      CanSender<::apollo::canbus::ArticulatedHunter>* const can_sender,
      MessageManager<::apollo::canbus::ArticulatedHunter>* const message_manager) override;
  bool Start() override;
  void Stop() override;
  Chassis chassis() override;
  void Emergency() override;
  ErrorCode EnableAutoMode() override;
  ErrorCode DisableAutoMode() override;
  ErrorCode EnableSteeringOnlyMode() override;
  ErrorCode EnableSpeedOnlyMode() override;
  void Gear(Chassis::GearPosition gear_position) override;
  void Brake(double brake) override;
  void Throttle(double throttle) override;
  void Speed(double speed) override;
  void Steer(double angle) override;
  void Steer(double angle, double angle_spd) override;
  void Acceleration(double acc) override;
  void SetEpbBreak(const ControlCommand& command) override;
  ErrorCode HandleCustomOperation(
      const external_command::ChassisCommand& command) override;
  void SetBeam(const VehicleSignal& signal) override;
  void SetHorn(const VehicleSignal& signal) override;
  void SetTurningSignal(const VehicleSignal& signal) override;
  bool VerifyID() override;

 private:
  void AddSendMessage();
  void ResetProtocol();
  bool CheckResponse(const int32_t flags, bool need_wait);
  bool CheckChassisError();
  void SecurityDogThreadFunc();
  void set_chassis_error_code(const Chassis::ErrorCode& error_code);
  Chassis::ErrorCode chassis_error_code();
  void set_chassis_error_mask(const int32_t mask);
  int32_t chassis_error_mask();

  // ====== 主车(Mycar) 控制协议指针 ======
  AcuDrivemotor563* drive_motor_563_ = nullptr;
  AcuEps547* eps_547_ = nullptr;

  // ====== 从车(Hunter) 控制协议指针 ======
  Controlmodeset421* control_mode_set_421_ = nullptr;
  Errorclearcommand441* error_clear_command_441_ = nullptr;
  Motioncommand111* motion_command_111_ = nullptr;

  // Thread
  std::unique_ptr<std::thread> thread_;

  // Mutex
  mutable std::mutex chassis_error_code_mutex_;
  mutable std::mutex chassis_mask_mutex_;

  // Error code
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;
  int32_t chassis_error_mask_ = 0;

  // Parameters
  common::VehicleParameter vehicle_params_;
  VehicleParameter params_;
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
