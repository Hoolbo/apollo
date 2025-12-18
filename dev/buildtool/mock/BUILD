
load("//tools/install:install.bzl", "install", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)


install(
    name = "mock_install",
    type = "neo",
    deps = ["//third_party/can_card_library:install","//third_party/camera_library:install","//modules/common_msgs:install","//cyber:install","//modules/common:install","//third_party/rtklib:install","//third_party/tf2:install","//modules/transform:install","//modules/drivers/canbus:install","//modules/canbus:install","//modules/canbus_vehicle/mycar:install"],
)


install_src_files(
    name = "mock_install_src",
    type = "neo",
    deps = ["//third_party/can_card_library:install_src","//third_party/camera_library:install_src","//modules/common_msgs:install_src","//cyber:install_src","//modules/common:install_src","//third_party/rtklib:install_src","//third_party/tf2:install_src","//modules/transform:install_src","//modules/drivers/canbus:install_src","//modules/canbus:install_src","//modules/canbus_vehicle/mycar:install_src"],
)
