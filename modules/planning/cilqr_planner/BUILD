load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

CILQR_COPTS = ['-DMODULE_NAME=\\"cilqr_planner\\"']

# ── CILQR algorithm library (no Apollo deps) ──

cc_library(
    name = "cilqr_lib",
    srcs = glob(["cilqr_lib/src/*.cpp"]),
    hdrs = glob(["cilqr_lib/include/*.h"]),
    includes = ["cilqr_lib/include"],
    copts = [
        "-std=c++17",
        "-Wno-error=unused-but-set-variable",
        "-Wno-error=unused-function",
        "-Wno-error=unused-result",
        "-Wno-error=sign-compare",
        "-Wno-error=reorder",
    ],
    deps = [
        "@eigen",
        "@opencv//:core",
        "@opencv//:imgproc",
        "@opencv//:imgcodecs",
    ],
)

# ── Component library ──

apollo_cc_library(
    name = "cilqr_planner_component_lib",
    srcs = ["cilqr_planner_component.cc"],
    hdrs = ["cilqr_planner_component.h"],
    copts = CILQR_COPTS + [
        "-Wno-error=reorder",
        "-Wno-error=unused-but-set-variable",
    ],
    alwayslink = True,
    deps = [
        ":cilqr_lib",
        "//modules/planning/cilqr_planner/proto:cilqr_planner_conf_cc_proto",
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
    ],
)

# ── Shared library for Cyber component loading ──

apollo_component(
    name = "libcilqr_planner_component.so",
    deps = [
        ":cilqr_planner_component_lib",
        "//modules/planning/cilqr_planner/proto:cilqr_planner_conf_cc_proto",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
        "data/**",
    ]) + ["cyberfile.xml"],
)

apollo_package()

cpplint()
