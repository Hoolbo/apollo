#include "modules/canbus_vehicle/articulated/protocol/front_eps_acu_556.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t FrontEpsAcu556::ID = 0x22C;

void FrontEpsAcu556::Parse(const std::uint8_t* bytes, int32_t length,
                           Articulated* chassis) const {
  chassis->mutable_front_eps_acu_556()->set_eps_angle(
      eps_angle(bytes, length));
  chassis->mutable_front_eps_acu_556()->set_eps_enable(
      eps_enable(bytes, length));
  chassis->mutable_front_eps_acu_556()->set_eps_error(
      eps_error(bytes, length));
  chassis->mutable_front_eps_acu_556()->set_eps_angle_speed(
      eps_angle_speed(bytes, length));
}

double FrontEpsAcu556::eps_angle(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte frame(bytes + 5);
  int32_t low = frame.get_byte(0, 8);
  Byte frame_high(bytes + 6);
  int32_t high = frame_high.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 1.0 - 1024.0;
}

bool FrontEpsAcu556::eps_enable(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(0);
}

int32_t FrontEpsAcu556::eps_error(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte frame(bytes + 0);
  return frame.get_byte(0, 8);
}

double FrontEpsAcu556::eps_angle_speed(const std::uint8_t* bytes,
                                       int32_t length) const {
  // DBC: 32|8@1+ (2.0,0) [0|180]
  Byte frame(bytes + 4);
  int32_t value = frame.get_byte(0, 8);
  return value * 2.0;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
