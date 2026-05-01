load("//tools/proto:proto.bzl", "proto_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "articulated_hunter_proto",
    srcs = ["articulated_hunter.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

apollo_package()
