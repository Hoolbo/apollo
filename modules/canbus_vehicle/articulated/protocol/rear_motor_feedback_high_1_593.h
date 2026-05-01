#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

// Hunter SE Motor Feedback High 1 (0x251 / 593)
// Motorola byte order
class RearMotorFeedbackHigh1_593
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

 private:
  // bit:7, signed, len:16, motorola, factor:1.0, unit:RPM
  int32_t motor1_speed(const std::uint8_t* bytes, int32_t length) const;
  // bit:23, signed, len:16, motorola, factor:0.1, unit:A
  double motor1_current(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
