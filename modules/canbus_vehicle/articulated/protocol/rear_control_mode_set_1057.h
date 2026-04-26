#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

// Hunter SE Control Mode Set (0x421 / 1057)
class RearControlModeSet1057
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  RearControlModeSet1057();

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  void set_mode(ArticulatedRearControlModeSet1057::ModeType mode) {
    mode_set_ = mode;
  }

 private:
  ArticulatedRearControlModeSet1057::ModeType mode_set_ =
      ArticulatedRearControlModeSet1057::STANDBY;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
