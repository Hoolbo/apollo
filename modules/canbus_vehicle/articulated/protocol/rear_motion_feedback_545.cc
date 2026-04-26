#include "modules/canbus_vehicle/articulated/protocol/rear_motion_feedback_545.h"

#include "glog/logging.h"

namespace apollo {
namespace canbus {
namespace articulated {

const int32_t RearMotionFeedback545::ID = 0x221;

void RearMotionFeedback545::Parse(const std::uint8_t* bytes, int32_t length,
                                  Articulated* chassis) const {
  chassis->mutable_rear_motion_feedback_545()->set_linear_speed(
      linear_speed(bytes, length));
  chassis->mutable_rear_motion_feedback_545()->set_steering_angle(
      steering_angle(bytes, length));
}

double RearMotionFeedback545::linear_speed(const std::uint8_t* bytes,
                                           int32_t length) const {
  // DBC: SG_ Linear_Speed : 7|16@0- (0.001,0) [-32768|32767] "m/s"
  // Motorola big-endian: Byte 0 = high, Byte 1 = low
  // Signed 16-bit
  int16_t raw = static_cast<int16_t>((bytes[0] << 8) | bytes[1]);
  return raw * 0.001;
}

double RearMotionFeedback545::steering_angle(const std::uint8_t* bytes,
                                             int32_t length) const {
  // DBC: SG_ Steering_Angle : 55|16@0- (0.001,0) [-32768|32767] "rad"
  // Motorola big-endian: start bit 55 -> Byte 6 = high, Byte 7 = low
  int16_t raw = static_cast<int16_t>((bytes[6] << 8) | bytes[7]);
  return raw * 0.001;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
