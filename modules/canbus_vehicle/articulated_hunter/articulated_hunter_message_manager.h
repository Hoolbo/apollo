#pragma once

#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/canbus_vehicle/articulated_hunter/proto/articulated_hunter.pb.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

class ArticulatedHunterMessageManager
    : public ::apollo::drivers::canbus::MessageManager<
          ::apollo::canbus::ArticulatedHunter> {
 public:
  ArticulatedHunterMessageManager();
  virtual ~ArticulatedHunterMessageManager();
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
