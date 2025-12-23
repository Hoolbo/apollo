
load("//tools/install:install.bzl", "install", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)


install(
    name = "mock_install",
    type = "neo",
    deps = ["//scripts:install","//third_party/tf2:install","//modules/common_msgs:install","//modules/calibration:install","//cyber:install","//modules/common:install","//modules/map:install","//modules/transform:install","//modules/dreamview:install","//modules/dreamview_plus:install"],
)


install_src_files(
    name = "mock_install_src",
    type = "neo",
    deps = ["//scripts:install_src","//third_party/tf2:install_src","//modules/common_msgs:install_src","//modules/calibration:install_src","//cyber:install_src","//modules/common:install_src","//modules/map:install_src","//modules/transform:install_src","//modules/dreamview:install_src","//modules/dreamview_plus:install_src"],
)
