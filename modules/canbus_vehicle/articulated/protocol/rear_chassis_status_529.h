#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

// Hunter SE Chassis Status (0x211 / 529)
// Motorola byte order
class RearChassisStatus529
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

 private:
  int32_t vehicle_state(const std::uint8_t* bytes, int32_t length) const;
  int32_t control_mode(const std::uint8_t* bytes, int32_t length) const;
  double battery_voltage(const std::uint8_t* bytes, int32_t length) const;
  int32_t fault_high(const std::uint8_t* bytes, int32_t length) const;
  int32_t fault_low(const std::uint8_t* bytes, int32_t length) const;
  int32_t message_count(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
