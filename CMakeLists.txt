cmake_minimum_required(VERSION 3.16)

project(dstar_lite)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Bonxai is checked out beside this package for host builds and copied to
# /opt/Bonxai by the full-stack Dockerfile.  Its core and probabilistic-map
# libraries are upstream CMake targets, not released ROS packages.
set(BONXAI_SOURCE_DIR "" CACHE PATH "Path to the Bonxai source checkout")
if(NOT BONXAI_SOURCE_DIR)
    set(BONXAI_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Bonxai")
endif()
if(NOT EXISTS "${BONXAI_SOURCE_DIR}/bonxai_core/CMakeLists.txt" OR
   NOT EXISTS "${BONXAI_SOURCE_DIR}/bonxai_map/CMakeLists.txt")
    message(FATAL_ERROR
        "Bonxai source is required. Set BONXAI_SOURCE_DIR to its checkout.")
endif()
add_subdirectory("${BONXAI_SOURCE_DIR}/bonxai_core" "${CMAKE_CURRENT_BINARY_DIR}/bonxai_core")
add_subdirectory("${BONXAI_SOURCE_DIR}/bonxai_map" "${CMAKE_CURRENT_BINARY_DIR}/bonxai_map")

# ---------------------------------------------------------------------------
# Core path-planning library
# ---------------------------------------------------------------------------

add_library(path_planning
    src/grid.cpp
    src/astar.cpp
    src/dstar_lite.cpp
    src/dstar_3d.cpp
)

target_include_directories(path_planning
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_options(path_planning
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

add_executable(dstar_lite_node
    src/dstar_lite_ros.cpp
)

# Do not use PRIVATE here.
# ament_target_dependencies() uses the plain target_link_libraries signature.
target_link_libraries(dstar_lite_node
    path_planning
)

ament_target_dependencies(dstar_lite_node
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    px4_msgs
)

target_compile_options(dstar_lite_node
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(pointcloud_bonxai_node
    src/pointcloud_bonxai.cpp
)

target_link_libraries(pointcloud_bonxai_node
    bonxai_map
    Eigen3::Eigen
)

ament_target_dependencies(pointcloud_bonxai_node
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
)

target_compile_options(pointcloud_bonxai_node
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(px4_lidar_tf_broadcaster_node
    src/px4_lidar_tf_broadcaster.cpp
)

target_link_libraries(px4_lidar_tf_broadcaster_node
    Eigen3::Eigen
)

ament_target_dependencies(px4_lidar_tf_broadcaster_node
    rclcpp
    geometry_msgs
    px4_msgs
    tf2_ros
)

target_compile_options(px4_lidar_tf_broadcaster_node
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(gazebo_lidar_tf_broadcaster_node
    src/gazebo_lidar_tf_broadcaster.cpp
)

target_link_libraries(gazebo_lidar_tf_broadcaster_node
    Eigen3::Eigen
    gz-transport13::core
    gz-msgs10::core
)

ament_target_dependencies(gazebo_lidar_tf_broadcaster_node
    rclcpp
    geometry_msgs
    tf2_ros
)

target_compile_options(gazebo_lidar_tf_broadcaster_node
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# A one-way Gazebo Harmonic PointCloudPacked -> ROS PointCloud2 bridge.  Keep
# it in-tree instead of depending on ros_gz_bridge, whose Gazebo ABI must
# match PX4's Harmonic install exactly.
add_executable(gazebo_pointcloud_bridge_node
    src/gazebo_pointcloud_bridge.cpp
)

target_link_libraries(gazebo_pointcloud_bridge_node
    gz-transport13::core
    gz-msgs10::core
)

ament_target_dependencies(gazebo_pointcloud_bridge_node
    rclcpp
    sensor_msgs
)

target_compile_options(gazebo_pointcloud_bridge_node
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# Gazebo Harmonic IMU -> ROS sensor_msgs/Imu bridge for simulated LIO.
add_executable(gazebo_imu_bridge_node
    src/gazebo_imu_bridge.cpp
)

target_link_libraries(gazebo_imu_bridge_node
    gz-transport13::core
    gz-msgs10::core
)

ament_target_dependencies(gazebo_imu_bridge_node
    rclcpp
    sensor_msgs
)

target_compile_options(gazebo_imu_bridge_node
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# These source files are optional in this checkout.  Keep the targets
# available when they are present, but do not make the rest of the package
# (including the Gazebo bridge launches) unbuildable when they are absent.
set(optional_turtlebot_targets)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/turtlebot_dstar_lite_ros.cpp")
    add_executable(turtlebot_dstar_lite_node
        src/turtlebot_dstar_lite_ros.cpp
    )

    target_link_libraries(turtlebot_dstar_lite_node
        path_planning
    )

    ament_target_dependencies(turtlebot_dstar_lite_node
        rclcpp
        nav_msgs
        sensor_msgs
        std_msgs
        geometry_msgs
        tf2
        tf2_geometry_msgs
    )

    target_compile_options(turtlebot_dstar_lite_node
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
    list(APPEND optional_turtlebot_targets turtlebot_dstar_lite_node)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/turtlebot_waypoint_controller.cpp")
    add_executable(turtlebot_waypoint_controller
        src/turtlebot_waypoint_controller.cpp
    )

    ament_target_dependencies(turtlebot_waypoint_controller
        rclcpp
        nav_msgs
        sensor_msgs
        geometry_msgs
        tf2
        tf2_geometry_msgs
    )

    target_compile_options(turtlebot_waypoint_controller
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
    list(APPEND optional_turtlebot_targets turtlebot_waypoint_controller)
endif()

# ---------------------------------------------------------------------------
# Test executables
# ---------------------------------------------------------------------------

add_executable(astar_test
    src/main.cpp
)

target_link_libraries(astar_test
    PRIVATE
        path_planning
)

add_executable(dstar_state_test
    src/dstar_state_test.cpp
)

target_link_libraries(dstar_state_test
    PRIVATE
        path_planning
)

add_executable(dstar_astar_test
    src/dstar_astar_test.cpp
)

target_link_libraries(dstar_astar_test
    PRIVATE
        path_planning
)

add_executable(pipeline_test
    src/pipeline_test.cpp
)

target_link_libraries(pipeline_test
    PRIVATE
        path_planning
)

add_executable(dstar_3d_test
    src/dstar_3d_test.cpp
)

target_link_libraries(dstar_3d_test
    PRIVATE
        path_planning
)

foreach(test_target
    astar_test
    dstar_state_test
    dstar_astar_test
    pipeline_test
    dstar_3d_test
)
    target_compile_options(${test_target}
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endforeach()

# ---------------------------------------------------------------------------
# Register assert-based tests with CTest
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
    enable_testing()

    add_test(
        NAME astar_tests
        COMMAND astar_test
    )

    add_test(
        NAME dstar_state_tests
        COMMAND dstar_state_test
    )

    add_test(
        NAME dstar_astar_tests
        COMMAND dstar_astar_test
    )

    add_test(
        NAME pipeline_tests
        COMMAND pipeline_test
    )

    add_test(
        NAME dstar_3d_tests
        COMMAND dstar_3d_test
    )
endif()

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS
        path_planning
        astar_test
        dstar_state_test
        dstar_astar_test
        pipeline_test
        dstar_3d_test
        dstar_lite_node
        pointcloud_bonxai_node
        px4_lidar_tf_broadcaster_node
        gazebo_lidar_tf_broadcaster_node
        gazebo_pointcloud_bridge_node
        gazebo_imu_bridge_node
        ${optional_turtlebot_targets}

    EXPORT export_path_planning

    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}

    INCLUDES DESTINATION include
)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(
    DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
)

ament_export_targets(
    export_path_planning
    HAS_LIBRARY_TARGET
)

ament_package()
