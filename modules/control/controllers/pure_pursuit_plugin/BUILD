load("//tools:apollo_package.bzl", "apollo_plugin", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_plugin(
    name = "libpure_pursuit_controller.so",
    srcs = ["pure_pursuit_controller.cc"],
    hdrs = ["pure_pursuit_controller.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"control\\\""],
    deps = [
        "//cyber",
        # 👇 核心依赖：ControlTask 基类
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/common/status",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

apollo_package()

cpplint()