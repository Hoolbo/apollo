#include "modules/canbus_vehicle/articulated_hunter/articulated_hunter_message_manager.h"

// Mycar protocol files (Main vehicle)
#include "modules/canbus_vehicle/articulated_hunter/protocol/acu_drivemotor_563.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/acu_eps_547.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/drivemotor_acu_572.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/eps_acu_556.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/vcu_acu_general_524.h"

// Hunter protocol files (Secondary vehicle)
#include "modules/canbus_vehicle/articulated_hunter/protocol/chassis_status_211.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/control_mode_set_421.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/error_clear_command_441.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motion_command_111.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motion_feedback_221.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motor_feedback_high_1_251.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motor_feedback_high_2_252.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motor_feedback_high_3_253.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motor_feedback_low_1_261.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motor_feedback_low_2_262.h"
#include "modules/canbus_vehicle/articulated_hunter/protocol/motor_feedback_low_3_263.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

ArticulatedHunterMessageManager::ArticulatedHunterMessageManager() {
  // ====== 主车(Mycar) 控制协议 ======
  AddSendProtocolData<AcuDrivemotor563, true>();
  AddSendProtocolData<AcuEps547, true>();

  // ====== 主车(Mycar) 反馈协议 ======
  AddRecvProtocolData<DrivemotorAcu572, true>();
  AddRecvProtocolData<EpsAcu556, true>();
  AddRecvProtocolData<VcuAcuGeneral524, true>();

  // ====== 从车(Hunter) 控制协议 ======
  AddSendProtocolData<MotionCommand111, true>();
  AddSendProtocolData<ControlModeSet421, true>();
  AddSendProtocolData<ErrorClearCommand441, true>();

  // ====== 从车(Hunter) 反馈协议 ======
  AddRecvProtocolData<ChassisStatus211, true>();
  AddRecvProtocolData<MotionFeedback221, true>();
  AddRecvProtocolData<MotorFeedbackHigh1251, true>();
  AddRecvProtocolData<MotorFeedbackHigh2252, true>();
  AddRecvProtocolData<MotorFeedbackHigh3253, true>();
  AddRecvProtocolData<MotorFeedbackLow1261, true>();
  AddRecvProtocolData<MotorFeedbackLow2262, true>();
  AddRecvProtocolData<MotorFeedbackLow3263, true>();
}

ArticulatedHunterMessageManager::~ArticulatedHunterMessageManager() {}

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
