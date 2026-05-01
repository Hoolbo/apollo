#include "modules/canbus_vehicle/articulated_hunter/protocol/vcu_acu_general_524.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t VcuAcuGeneral524::ID = 0x20C;

void VcuAcuGeneral524::Parse(const std::uint8_t* bytes, int32_t length,
                             ::apollo::canbus::ArticulatedHunter* chassis) const {
  chassis->mutable_vcu_acu_general_524()->set_acu_error(
      acu_error(bytes, length));
  chassis->mutable_vcu_acu_general_524()->set_acu_remote_control(
      acu_remote_control(bytes, length));
  chassis->mutable_vcu_acu_general_524()->set_acu_receive_info(
      acu_receive_info(bytes, length));
  chassis->mutable_vcu_acu_general_524()->set_acu_control_mode(
      acu_control_mode(bytes, length));
}

bool VcuAcuGeneral524::acu_error(const std::uint8_t* bytes,
                                 int32_t length) const {
  // SG_ ACU_Error : 0|1@1+
  Byte frame(bytes + 0);
  return frame.is_bit_1(0);
}

bool VcuAcuGeneral524::acu_remote_control(const std::uint8_t* bytes,
                                          int32_t length) const {
  // SG_ ACU_Remote_control : 62|1@1+ (Byte 7, Bit 6)
  Byte frame(bytes + 7);
  return frame.is_bit_1(6);
}

bool VcuAcuGeneral524::acu_control_mode(const std::uint8_t* bytes,
                                        int32_t length) const {
  // SG_ ACU_Control_Mode : 63|1@1+ (Byte 7, Bit 7)
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

bool VcuAcuGeneral524::acu_receive_info(const std::uint8_t* bytes,
                                        int32_t length) const {
  // SG_ ACU_Receive_info : 59|1@1+ (Byte 7, Bit 3)
  Byte frame(bytes + 7);
  return frame.is_bit_1(3);
}

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
