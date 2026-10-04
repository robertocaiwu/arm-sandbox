cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# Eigen/Pinocchio math is far slower unoptimized; colcon passes no build type by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_sandbox_kinematics REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_eigen REQUIRED)

# ROS-free control core: dynamics + task-space control laws.
set(CONTROL_CORE_SOURCES src/robot_dynamics.cpp)
add_library(control_core SHARED ${CONTROL_CORE_SOURCES})
target_include_directories(control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(control_core PUBLIC
  arm_sandbox_kinematics::arm_sandbox_kinematics Eigen3::Eigen pinocchio::pinocchio)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS control_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(arm_sandbox_kinematics Eigen3 pinocchio)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(arm_sandbox_description REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)

  # Same approach as arm_sandbox_kinematics: test on the real robot's URDF and robot.yaml.
  set(TEST_ROBOT panda)
  set(TEST_ROBOT_DIR "${arm_sandbox_description_DIR}/../${TEST_ROBOT}")
  set(TEST_URDF "${CMAKE_CURRENT_BINARY_DIR}/${TEST_ROBOT}.urdf")
  add_custom_command(
    OUTPUT ${TEST_URDF}
    COMMAND ${XACRO_EXECUTABLE} ${TEST_ROBOT_DIR}/urdf/${TEST_ROBOT}.urdf.xacro -o ${TEST_URDF}
    DEPENDS ${TEST_ROBOT_DIR}/urdf/${TEST_ROBOT}.urdf.xacro
    COMMENT "Generating ${TEST_ROBOT}.urdf for the control tests")
  add_custom_target(test_urdf ALL DEPENDS ${TEST_URDF})
  set(TEST_DEFINITIONS TEST_URDF_PATH="${TEST_URDF}" TEST_ROBOT_CONFIG_PATH="${TEST_ROBOT_DIR}/config/robot.yaml")

  foreach(test_name test_robot_dynamics)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    add_dependencies(${test_name} test_urdf)
    target_link_libraries(${test_name} control_core yaml-cpp)
    target_compile_definitions(${test_name} PRIVATE ${TEST_DEFINITIONS})
  endforeach()

endif()

ament_package()
