load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_articulated",
    srcs = [
        "articulated_controller.cc",
        "articulated_message_manager.cc",
        "articulated_vehicle_factory.cc",
        "protocol/front_acu_drivemotor_563.cc",
        "protocol/front_acu_eps_547.cc",
        "protocol/front_drivemotor_acu_572.cc",
        "protocol/front_eps_acu_556.cc",
        "protocol/front_vcu_acu_general_524.cc",
        "protocol/rear_chassis_status_529.cc",
        "protocol/rear_control_mode_set_1057.cc",
        "protocol/rear_motion_command_273.cc",
        "protocol/rear_motion_feedback_545.cc",
    ],
    hdrs = [
        "articulated_controller.h",
        "articulated_message_manager.h",
        "articulated_vehicle_factory.h",
        "protocol/front_acu_drivemotor_563.h",
        "protocol/front_acu_eps_547.h",
        "protocol/front_drivemotor_acu_572.h",
        "protocol/front_eps_acu_556.h",
        "protocol/front_vcu_acu_general_524.h",
        "protocol/rear_chassis_status_529.h",
        "protocol/rear_control_mode_set_1057.h",
        "protocol/rear_motion_command_273.h",
        "protocol/rear_motion_feedback_545.h",
    ],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/articulated/proto:articulated_cc_proto",
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
    name = "libarticulated_vehicle_factory.so",
    deps = [":apollo_canbus_vehicle_articulated"],
)

apollo_package()

cpplint()
