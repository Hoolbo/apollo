#pragma once

#include "modules/canbus_vehicle/articulated_hunter/proto/articulated_hunter.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

class DrivemotorAcu572
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::ArticulatedHunter> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ArticulatedHunter* chassis) const override;

 private:
  double drive_motor_speed(const std::uint8_t* bytes, int32_t length) const;
  double drive_motor_torque(const std::uint8_t* bytes, int32_t length) const;
  ArticulatedHunterDrivemotorAcu572::ShiftType drive_motor_shift(const std::uint8_t* bytes,
                                                     int32_t length) const;
  bool drive_motor_enable(const std::uint8_t* bytes, int32_t length) const;
  bool drive_motor_reply(const std::uint8_t* bytes, int32_t length) const;
  bool drive_motor_mode(const std::uint8_t* bytes, int32_t length) const;
  bool drive_motor_lock(const std::uint8_t* bytes, int32_t length) const;
  bool drive_motor_stop(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
