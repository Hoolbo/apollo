#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

// Hunter SE Motor Feedback High 3 (0x253 / 595)
class RearMotorFeedbackHigh3_595
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

 private:
  int32_t motor3_speed(const std::uint8_t* bytes, int32_t length) const;
  double motor3_current(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
