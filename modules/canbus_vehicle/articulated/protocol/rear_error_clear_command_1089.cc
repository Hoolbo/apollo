#include "modules/canbus_vehicle/articulated/protocol/rear_error_clear_command_1089.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t RearErrorClearCommand1089::ID = 0x441;

RearErrorClearCommand1089::RearErrorClearCommand1089() { Reset(); }

uint32_t RearErrorClearCommand1089::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void RearErrorClearCommand1089::Parse(const std::uint8_t* bytes,
                                      int32_t length,
                                      Articulated* chassis) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);
  chassis->mutable_rear_error_clear_command_1089()->set_clear_code(
      static_cast<ArticulatedRearErrorClearCommand1089::ClearCodeType>(x));
}

void RearErrorClearCommand1089::UpdateData(uint8_t* data) {
  int x = clear_code_;
  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

void RearErrorClearCommand1089::Reset() {
  clear_code_ = ArticulatedRearErrorClearCommand1089::CLEAR_TURN_FAULT;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
