load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

CILQR_COPTS = ['-DMODULE_NAME=\\"cilqr_planner\\"']

# ── Proto ──

proto_library(
    name = "cilqr_planner_conf_proto",
    srcs = ["proto/cilqr_planner_conf.proto"],
)

cc_proto_library(
    name = "cilqr_planner_conf_cc_proto",
    deps = [":cilqr_planner_conf_proto"],
)

# ── CILQR algorithm library (ported from planner/, no Apollo deps) ──

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

cc_library(
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
        ":cilqr_planner_conf_cc_proto",
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

cc_binary(
    name = "libcilqr_planner_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":cilqr_planner_component_lib"],
)

# ── Runtime data ──

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
        "data/**",
    ]) + ["cyberfile.xml"],
)

# ── Install targets (required by buildtool) ──

install(
    name = "_install_lib",
    targets = [":libcilqr_planner_component.so"],
    data = ["cyberfile.xml"],
    library_dest = "lib/modules/planning/cilqr_planner",
    type = "neo",
    package_path = "modules/planning/cilqr_planner",
    visibility = ["//visibility:public"],
)

install(
    name = "_install_data",
    data = [":runtime_data"],
    data_dest = "share/modules/planning/cilqr_planner",
    type = "neo",
    package_path = "modules/planning/cilqr_planner",
    visibility = ["//visibility:public"],
)

install(
    name = "install",
    deps = [":_install_lib", ":_install_data"],
    type = "neo",
    package_path = "modules/planning/cilqr_planner",
    visibility = ["//visibility:public"],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "src/modules/planning/cilqr_planner",
    filter = "*",
    type = "neo",
    visibility = ["//visibility:public"],
)
