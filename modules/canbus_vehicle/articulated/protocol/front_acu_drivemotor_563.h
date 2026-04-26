#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

class FrontAcuDrivemotor563
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  FrontAcuDrivemotor563();

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // Setters
  void set_drive_motor_torque(double torque) { drive_motor_torque_ = torque; }
  void set_drive_motor_speed(double speed) { drive_motor_speed_ = speed; }
  void set_drive_motor_shift(
      ArticulatedFrontAcuDrivemotor563::ShiftType shift) {
    drive_motor_shift_ = shift;
  }
  void set_drive_motor_mode(bool mode) { drive_motor_mode_ = mode ? 1 : 0; }
  void set_drive_motor_enable(bool enable) {
    drive_motor_enable_ = enable ? 1 : 0;
  }

 private:
  void set_acu_drivemotor_torque(uint8_t* data, double torque);
  void set_acu_drivemotor_speed(uint8_t* data, double speed);
  void set_acu_drivemotor_select_shift(
      uint8_t* data, ArticulatedFrontAcuDrivemotor563::ShiftType shift);
  void set_acu_drivemotor_select_mode(uint8_t* data, int mode);
  void set_acu_drivemotor_select_enable(uint8_t* data, int enable);

  double drive_motor_torque_ = 0.0;
  double drive_motor_speed_ = 0.0;
  ArticulatedFrontAcuDrivemotor563::ShiftType drive_motor_shift_ =
      ArticulatedFrontAcuDrivemotor563::SHIFT_P;
  int drive_motor_mode_ = 0;
  int drive_motor_enable_ = 0;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
