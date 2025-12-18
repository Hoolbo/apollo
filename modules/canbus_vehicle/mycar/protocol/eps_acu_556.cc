#include "modules/canbus_vehicle/mycar/protocol/eps_acu_556.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mycar {

using ::apollo::drivers::canbus::Byte;

const int32_t EpsAcu556::ID = 0x22C;

void EpsAcu556::Parse(const std::uint8_t* bytes, int32_t length,
                      Mycar* chassis) const {
  chassis->mutable_eps_acu_556()->set_eps_angle(eps_angle(bytes, length));
  chassis->mutable_eps_acu_556()->set_eps_enable(eps_enable(bytes, length));
}

double EpsAcu556::eps_angle(const std::uint8_t* bytes, int32_t length) const {
  // ROS code: `Data[5] + Data[6] * 16 * 16 - 1024`
  Byte frame(bytes + 5);
  int32_t low = frame.get_byte(0, 8);
  Byte frame_high(bytes + 6);
  int32_t high = frame_high.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 1.0 - 1024.0;
}

bool EpsAcu556::eps_enable(const std::uint8_t* bytes, int32_t length) const {
  // DBC: 56|1@1+ (Byte 7 bit 0)
  Byte frame(bytes + 7);
  return frame.is_bit_1(0);
}

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
