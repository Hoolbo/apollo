cmake_minimum_required(VERSION 3.18)
project(planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- ament & ROS deps ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- Eigen3 ----
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- OpenCV ----
find_package(OpenCV REQUIRED)

# ---- nlohmann_json (header-only fallback) ----
find_package(nlohmann_json QUIET)

# ---- include dirs ----
include_directories(include)

# ---- Cilqr library (no matplotlib, no Python) ----
set(CILQR_SOURCES
    src/ilqr.cpp
    src/utils.cpp
    src/articulated_hybrid_astar.cpp
    src/ompl_rs.cpp
    src/config_loader.cpp
)

add_library(cilqr_lib STATIC ${CILQR_SOURCES})
target_include_directories(cilqr_lib PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cilqr_lib PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
if(TARGET nlohmann_json::nlohmann_json)
    target_link_libraries(cilqr_lib PUBLIC nlohmann_json::nlohmann_json)
endif()

# ---- ROS2 planner node ----
add_executable(planner_node src/planner_node.cpp)
target_include_directories(planner_node PUBLIC include)
target_link_libraries(planner_node cilqr_lib)
ament_target_dependencies(planner_node
    rclcpp
    nav_msgs
    geometry_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# ---- install ----
install(TARGETS planner_node
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS scripts/obstacle_publisher.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
