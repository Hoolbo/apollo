#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus_vehicle/mycar/proto/mycar.pb.h"

namespace apollo {
namespace canbus {
namespace mycar {

class AcuDrivemotor563 : public ::apollo::drivers::canbus::ProtocolData<
                             ::apollo::canbus::Mycar> {
 public:
  static const int32_t ID;

  AcuDrivemotor563();
  
  void Parse(const std::uint8_t* bytes, int32_t length,
             Mycar* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // Setters
  void set_drive_motor_speed(double speed) { drive_motor_speed_ = speed; }
  void set_drive_motor_shift(MycarAcuDrivemotor563::ShiftType shift) { drive_motor_shift_ = shift; }
  void set_drive_motor_mode(bool mode) { drive_motor_mode_ = mode ? 1 : 0; }
  void set_drive_motor_enable(bool enable) { drive_motor_enable_ = enable ? 1 : 0; }

 private:
  void set_acu_drivemotor_speed(uint8_t* data, double speed);
  void set_acu_drivemotor_select_shift(uint8_t* data, MycarAcuDrivemotor563::ShiftType shift);
  void set_acu_drivemotor_select_mode(uint8_t* data, int mode);
  void set_acu_drivemotor_select_enable(uint8_t* data, int enable);

  double drive_motor_speed_ = 0.0;
  MycarAcuDrivemotor563::ShiftType drive_motor_shift_ = MycarAcuDrivemotor563::SHIFT_P;
  int drive_motor_mode_ = 0;
  int drive_motor_enable_ = 0;
};

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
