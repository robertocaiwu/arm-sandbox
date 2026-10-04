cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_tasks)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_sandbox_kinematics REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# ROS-free task logic (task files, pose errors, move timing), unit-tested on its own.
add_library(reach_task STATIC src/reach_task.cpp)
set_target_properties(reach_task PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(reach_task PUBLIC include)
target_link_libraries(reach_task PUBLIC Eigen3::Eigen yaml-cpp)

# Thin ROS node on top of it.
add_executable(reach_runner src/reach_runner.cpp)
target_link_libraries(reach_runner reach_task arm_sandbox_kinematics::arm_sandbox_kinematics)
ament_target_dependencies(reach_runner control_msgs rclcpp rclcpp_action sensor_msgs std_msgs tf2_eigen tf2_ros)

install(TARGETS reach_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_reach_task test/test_reach_task.cpp)
  target_link_libraries(test_reach_task reach_task)
  target_compile_definitions(test_reach_task PRIVATE
    REACH_TASK_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/tasks/reach.yaml")
  # Full stack: sim + reach_runner. Own ROS domain (see arm_sandbox_bringup/CMakeLists.txt).
  add_launch_test(test/test_reach.py TIMEOUT 300 ENV ROS_DOMAIN_ID=45)
endif()

ament_package()
