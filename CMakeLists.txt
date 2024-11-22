cmake_minimum_required(VERSION 3.10)
project(your_project)

# Enable testing
enable_testing()

# Find or fetch GTest
find_package(GTest REQUIRED)

# Find Eigen package
find_package(Eigen3 REQUIRED)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add test executable
add_executable(behavior_planner_test src/tests/behavior_planner_test.cpp)
target_link_libraries(behavior_planner_test
    PRIVATE
    GTest::GTest
    GTest::Main
    GTest::gmock
    GTest::gmock_main
    ${OpenCV_LIBS}
)

target_include_directories(behavior_planner_test PRIVATE
    ${PROJECT_SOURCE_DIR}/src/core/common
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/core
    ${PROJECT_SOURCE_DIR}/src/core/sensors
    ${PROJECT_SOURCE_DIR}/src/tests
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/include
) 