#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

// Hunter SE Error Clear Command (0x441 / 1089)
// Motorola byte order
class RearErrorClearCommand1089
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  RearErrorClearCommand1089();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  void set_clear_code(
      ArticulatedRearErrorClearCommand1089::ClearCodeType clear_code) {
    clear_code_ = clear_code;
  }

 private:
  ArticulatedRearErrorClearCommand1089::ClearCodeType clear_code_ =
      ArticulatedRearErrorClearCommand1089::CLEAR_TURN_FAULT;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
