cmake_minimum_required(VERSION 3.16)

project(dstar_lite)

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

# ---------------------------------------------------------------------------
# Compiler configuration
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Core path-planning library
# ---------------------------------------------------------------------------

add_library(path_planning
    src/grid.cpp
    src/astar.cpp
    src/dstar_lite.cpp
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

foreach(test_target
    astar_test
    dstar_state_test
    dstar_astar_test
    pipeline_test
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
        dstar_lite_node
        turtlebot_dstar_lite_node

    EXPORT export_path_planning

    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}

    INCLUDES DESTINATION include
)

ament_export_targets(
    export_path_planning
    HAS_LIBRARY_TARGET
)

ament_package()
