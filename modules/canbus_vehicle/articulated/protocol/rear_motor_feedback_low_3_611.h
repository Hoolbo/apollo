#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

// Hunter SE Motor Feedback Low 3 (0x263 / 611)
class RearMotorFeedbackLow3_611
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

 private:
  double driver_voltage_3(const std::uint8_t* bytes, int32_t length) const;
  int32_t driver_temp_3(const std::uint8_t* bytes, int32_t length) const;
  int32_t motor_temp_3(const std::uint8_t* bytes, int32_t length) const;
  int32_t driver_status_3(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
