#include "modules/canbus_vehicle/articulated/protocol/front_acu_drivemotor_563.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t FrontAcuDrivemotor563::ID = 0x233;

FrontAcuDrivemotor563::FrontAcuDrivemotor563() { Reset(); }

void FrontAcuDrivemotor563::Parse(const std::uint8_t* bytes, int32_t length,
                                  Articulated* chassis) const {}

void FrontAcuDrivemotor563::UpdateData(uint8_t* data) {
  AINFO << "FrontAcuDrivemotor563::UpdateData called, current speed_: "
        << drive_motor_speed_;

  set_acu_drivemotor_speed(data, drive_motor_speed_);
  set_acu_drivemotor_torque(data, drive_motor_torque_);
  set_acu_drivemotor_select_shift(data, drive_motor_shift_);
  set_acu_drivemotor_select_mode(data, drive_motor_mode_);
  set_acu_drivemotor_select_enable(data, drive_motor_enable_);
}

void FrontAcuDrivemotor563::Reset() {
  drive_motor_speed_ = 0.0;
  drive_motor_torque_ = 0.0;
  drive_motor_shift_ = ArticulatedFrontAcuDrivemotor563::SHIFT_P;
  drive_motor_mode_ = 0;
  drive_motor_enable_ = 0;
}

void FrontAcuDrivemotor563::set_acu_drivemotor_speed(uint8_t* data,
                                                     double speed) {
  // DBC: 40|16@1+ (0.1,0) [0|1000] "km/h"
  speed = ProtocolData::BoundedValue(0.0, 1000.0, speed);
  int32_t x = static_cast<int32_t>(speed * 10.0);

  Byte frame(data + 5);
  frame.set_value(x, 0, 8);
  Byte frame_high(data + 6);
  frame_high.set_value(x >> 8, 0, 8);
}

void FrontAcuDrivemotor563::set_acu_drivemotor_torque(uint8_t* data,
                                                      double torque) {
  // DBC: 24|16@1+ (1,0) [0|0]
  torque = ProtocolData::BoundedValue(0.0, 65535.0, torque);
  int32_t x = static_cast<int32_t>(torque);

  Byte frame(data + 3);
  frame.set_value(x, 0, 8);
  Byte frame_high(data + 4);
  frame_high.set_value(x >> 8, 0, 8);
}

void FrontAcuDrivemotor563::set_acu_drivemotor_select_shift(
    uint8_t* data, ArticulatedFrontAcuDrivemotor563::ShiftType shift) {
  // DBC: 60|2@1+ (1,0) [0|3]
  int32_t x = static_cast<int32_t>(shift);
  Byte frame(data + 7);
  frame.set_value(x, 4, 2);
}

void FrontAcuDrivemotor563::set_acu_drivemotor_select_mode(uint8_t* data,
                                                           int mode) {
  // DBC: 57|1@1+
  Byte frame(data + 7);
  frame.set_value(mode, 1, 1);
}

void FrontAcuDrivemotor563::set_acu_drivemotor_select_enable(uint8_t* data,
                                                             int enable) {
  // DBC: 56|1@1+
  Byte frame(data + 7);
  frame.set_value(enable, 0, 1);
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
