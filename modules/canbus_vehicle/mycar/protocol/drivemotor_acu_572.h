#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus_vehicle/mycar/proto/mycar.pb.h"

namespace apollo {
namespace canbus {
namespace mycar {

class DrivemotorAcu572 : public ::apollo::drivers::canbus::ProtocolData<
                             ::apollo::canbus::Mycar> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Mycar* chassis) const override;

 private:
  double drive_motor_speed(const std::uint8_t* bytes, int32_t length) const;
  double drive_motor_torque(const std::uint8_t* bytes, int32_t length) const;
  MycarDrivemotorAcu572::ShiftType drive_motor_shift(const std::uint8_t* bytes, int32_t length) const;
  bool drive_motor_enable(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
