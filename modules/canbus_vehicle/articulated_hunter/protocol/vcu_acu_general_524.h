#pragma once

#include "modules/canbus_vehicle/articulated_hunter/proto/articulated_hunter.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

class VcuAcuGeneral524
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::ArticulatedHunter> {
 public:
  static const int32_t ID;
  void Parse(const std::uint8_t* bytes, int32_t length,
             ::apollo::canbus::ArticulatedHunter* chassis) const override;

 private:
  bool acu_error(const std::uint8_t* bytes, int32_t length) const;
  bool acu_remote_control(const std::uint8_t* bytes, int32_t length) const;
  bool acu_control_mode(const std::uint8_t* bytes, int32_t length) const;
  bool acu_receive_info(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
