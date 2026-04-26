#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

class FrontVcuAcuGeneral524
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;
  void Parse(const std::uint8_t* bytes, int32_t length,
             ::apollo::canbus::Articulated* chassis) const override;

 private:
  bool acu_error(const std::uint8_t* bytes, int32_t length) const;
  bool acu_remote_control(const std::uint8_t* bytes, int32_t length) const;
  bool acu_control_mode(const std::uint8_t* bytes, int32_t length) const;
  bool acu_receive_info(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
