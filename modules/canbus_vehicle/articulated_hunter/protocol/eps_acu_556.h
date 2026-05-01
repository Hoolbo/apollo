#pragma once

#include "modules/canbus_vehicle/articulated_hunter/proto/articulated_hunter.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

class EpsAcu556
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::ArticulatedHunter> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ArticulatedHunter* chassis) const override;

 private:
  double eps_angle(const std::uint8_t* bytes, int32_t length) const;
  bool eps_enable(const std::uint8_t* bytes, int32_t length) const;
  int32_t eps_error(const std::uint8_t* bytes, int32_t length) const;
  double eps_angle_speed(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
