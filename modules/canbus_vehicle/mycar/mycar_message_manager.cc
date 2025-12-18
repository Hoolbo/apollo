#include "modules/canbus_vehicle/mycar/mycar_message_manager.h"

#include "modules/canbus_vehicle/mycar/protocol/acu_drivemotor_563.h"
#include "modules/canbus_vehicle/mycar/protocol/acu_eps_547.h"
#include "modules/canbus_vehicle/mycar/protocol/drivemotor_acu_572.h"
#include "modules/canbus_vehicle/mycar/protocol/eps_acu_556.h"

namespace apollo {
namespace canbus {
namespace mycar {

MycarMessageManager::MycarMessageManager() {
  // Control Messages
  AddSendProtocolData<AcuDrivemotor563, true>();
  AddSendProtocolData<AcuEps547, true>();

  // Feedback Messages
  AddRecvProtocolData<DrivemotorAcu572, true>();
  AddRecvProtocolData<EpsAcu556, true>();
}

MycarMessageManager::~MycarMessageManager() {}

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
