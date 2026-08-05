cmake_minimum_required(VERSION 3.16)

project(dstar_lite)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Grid and planning algorithms
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

# Test executable
add_executable(astar_test
    src/main.cpp
)

add_executable(dstar_state_test
    src/dstar_state_test.cpp
)

add_executable(dstar_astar_test
    src/dstar_astar_test.cpp
)

target_link_libraries( astar_test
    PRIVATE
        path_planning
)

target_link_libraries( dstar_state_test
    PRIVATE
        path_planning
)

target_link_libraries( dstar_astar_test
    PRIVATE
        path_planning
)

target_compile_options(astar_test
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

target_compile_options(dstar_state_test
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)


target_compile_options(dstar_astar_test
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# Register the current assert-based test with CTest.
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
    
endif()

# Install headers.
install(
    DIRECTORY include/
    DESTINATION include
)

# Install the library and executable.
install(
    TARGETS
        path_planning
        astar_test
        dstar_state_test
        dstar_astar_test
    EXPORT export_path_planning
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_export_targets(export_path_planning HAS_LIBRARY_TARGET)

ament_package()