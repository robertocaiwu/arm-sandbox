cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_viz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
# Built and installed by scripts/install_deps.sh; not an apt/rosdep package.
set(RERUN_CPP_SDK_PREFIX "/opt/rerun_cpp_sdk" CACHE PATH "Rerun C++ SDK install prefix")
find_package(rerun_sdk REQUIRED HINTS ${RERUN_CPP_SDK_PREFIX})

add_executable(rerun_bridge src/main.cpp src/rerun_bridge.cpp)
target_include_directories(rerun_bridge PRIVATE include)
ament_target_dependencies(rerun_bridge rclcpp sensor_msgs std_msgs tf2_msgs)
# Plain signature: ament_target_dependencies uses it too, and CMake forbids mixing the two.
target_link_libraries(rerun_bridge rerun_sdk)

install(TARGETS rerun_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_rerun_bridge.py TIMEOUT 120 ENV ROS_DOMAIN_ID=43)
endif()

ament_package()
