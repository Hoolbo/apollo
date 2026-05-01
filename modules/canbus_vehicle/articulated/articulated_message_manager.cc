#include "modules/canbus_vehicle/articulated/articulated_message_manager.h"

// Front vehicle protocols (FCBee2, Intel byte order)
#include "modules/canbus_vehicle/articulated/protocol/front_acu_drivemotor_563.h"
#include "modules/canbus_vehicle/articulated/protocol/front_acu_eps_547.h"
#include "modules/canbus_vehicle/articulated/protocol/front_drivemotor_acu_572.h"
#include "modules/canbus_vehicle/articulated/protocol/front_eps_acu_556.h"
#include "modules/canbus_vehicle/articulated/protocol/front_vcu_acu_general_524.h"

// Rear vehicle protocols (Hunter SE, Motorola byte order)
#include "modules/canbus_vehicle/articulated/protocol/rear_chassis_status_529.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_control_mode_set_1057.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_error_clear_command_1089.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motion_command_273.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motion_feedback_545.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_high_1_593.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_high_2_594.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_high_3_595.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_low_1_609.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_low_2_610.h"
#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_low_3_611.h"

namespace apollo {
namespace canbus {
namespace articulated {

ArticulatedMessageManager::ArticulatedMessageManager() {
  // ---- Front vehicle: Send protocols ----
  AddSendProtocolData<FrontAcuDrivemotor563, true>();
  AddSendProtocolData<FrontAcuEps547, true>();

  // ---- Rear vehicle: Send protocols ----
  AddSendProtocolData<RearMotionCommand273, true>();
  AddSendProtocolData<RearControlModeSet1057, true>();
  AddSendProtocolData<RearErrorClearCommand1089, true>();

  // ---- Front vehicle: Receive protocols ----
  AddRecvProtocolData<FrontDrivemotorAcu572, true>();
  AddRecvProtocolData<FrontEpsAcu556, true>();
  AddRecvProtocolData<FrontVcuAcuGeneral524, true>();

  // ---- Rear vehicle: Receive protocols ----
  AddRecvProtocolData<RearMotionFeedback545, true>();
  AddRecvProtocolData<RearChassisStatus529, true>();
  AddRecvProtocolData<RearMotorFeedbackHigh1_593, true>();
  AddRecvProtocolData<RearMotorFeedbackHigh2_594, true>();
  AddRecvProtocolData<RearMotorFeedbackHigh3_595, true>();
  AddRecvProtocolData<RearMotorFeedbackLow1_609, true>();
  AddRecvProtocolData<RearMotorFeedbackLow2_610, true>();
  AddRecvProtocolData<RearMotorFeedbackLow3_611, true>();
}

ArticulatedMessageManager::~ArticulatedMessageManager() {}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
