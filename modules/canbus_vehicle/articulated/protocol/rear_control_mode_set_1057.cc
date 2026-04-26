#include "modules/canbus_vehicle/articulated/protocol/rear_control_mode_set_1057.h"

#include "glog/logging.h"

namespace apollo {
namespace canbus {
namespace articulated {

const int32_t RearControlModeSet1057::ID = 0x421;

RearControlModeSet1057::RearControlModeSet1057() { Reset(); }

void RearControlModeSet1057::Parse(const std::uint8_t* bytes, int32_t length,
                                   Articulated* chassis) const {
  // This is a send-only message, no parsing needed
}

void RearControlModeSet1057::UpdateData(uint8_t* data) {
  // DBC: SG_ Mode_Set : 7|8@0+ (1,0) [0|2]
  // Motorola big-endian: start bit 7, length 8 -> Byte 0
  // 0=Standby, 1=CanControl, 2=ToStandby
  data[0] = static_cast<uint8_t>(mode_set_);
  AINFO << "RearControlModeSet1057: mode=" << static_cast<int>(mode_set_);
}

void RearControlModeSet1057::Reset() {
  mode_set_ = ArticulatedRearControlModeSet1057::STANDBY;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
