#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

// Hunter SE Motor Feedback Low 1 (0x261 / 609)
class RearMotorFeedbackLow1_609
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

 private:
  // bit:7, unsigned, len:16, motorola, factor:0.1, unit:V
  double driver_voltage_1(const std::uint8_t* bytes, int32_t length) const;
  // bit:23, signed, len:16, motorola, factor:1.0, unit:degC
  int32_t driver_temp_1(const std::uint8_t* bytes, int32_t length) const;
  // bit:39, signed, len:8, motorola, factor:1.0, unit:degC
  int32_t motor_temp_1(const std::uint8_t* bytes, int32_t length) const;
  // bit:47, unsigned, len:8, motorola, factor:1.0
  int32_t driver_status_1(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
