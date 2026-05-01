load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_articulated_hunter",
    srcs = [
        "articulated_hunter_controller.cc",
        "articulated_hunter_message_manager.cc",
        "articulated_hunter_vehicle_factory.cc",
        # Mycar protocol files (Main vehicle)
        "protocol/acu_drivemotor_563.cc",
        "protocol/acu_eps_547.cc",
        "protocol/drivemotor_acu_572.cc",
        "protocol/eps_acu_556.cc",
        "protocol/vcu_acu_general_524.cc",
        # Hunter protocol files (Secondary vehicle)
        "protocol/chassis_status_211.cc",
        "protocol/control_mode_set_421.cc",
        "protocol/error_clear_command_441.cc",
        "protocol/motion_command_111.cc",
        "protocol/motion_feedback_221.cc",
        "protocol/motor_feedback_high_1_251.cc",
        "protocol/motor_feedback_high_2_252.cc",
        "protocol/motor_feedback_high_3_253.cc",
        "protocol/motor_feedback_low_1_261.cc",
        "protocol/motor_feedback_low_2_262.cc",
        "protocol/motor_feedback_low_3_263.cc",
    ],
    hdrs = [
        "articulated_hunter_controller.h",
        "articulated_hunter_message_manager.h",
        "articulated_hunter_vehicle_factory.h",
        # Mycar protocol headers (Main vehicle)
        "protocol/acu_drivemotor_563.h",
        "protocol/acu_eps_547.h",
        "protocol/drivemotor_acu_572.h",
        "protocol/eps_acu_556.h",
        "protocol/vcu_acu_general_524.h",
        # Hunter protocol headers (Secondary vehicle)
        "protocol/chassis_status_211.h",
        "protocol/control_mode_set_421.h",
        "protocol/error_clear_command_441.h",
        "protocol/motion_command_111.h",
        "protocol/motion_feedback_221.h",
        "protocol/motor_feedback_high_1_251.h",
        "protocol/motor_feedback_high_2_252.h",
        "protocol/motor_feedback_high_3_253.h",
        "protocol/motor_feedback_low_1_261.h",
        "protocol/motor_feedback_low_2_262.h",
        "protocol/motor_feedback_low_3_263.h",
    ],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/articulated_hunter/proto:articulated_hunter_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_id_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/common_msgs/external_command_msgs:chassis_command_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libarticulated_hunter_vehicle_factory.so",
    deps = [":apollo_canbus_vehicle_articulated_hunter"],
)

apollo_package()

cpplint()
