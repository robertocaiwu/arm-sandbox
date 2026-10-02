cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# Eigen-heavy math is ~10x slower unoptimized; colcon passes no build type by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

# ROS-free library: only Eigen and urdfdom (design spec "Kinematics").
add_library(${PROJECT_NAME} SHARED src/kinematic_chain.cpp src/ik.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(${PROJECT_NAME} PRIVATE ${urdfdom_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen PRIVATE urdfdom::urdf_parser)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 urdfdom)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(arm_sandbox_description REQUIRED)
  find_package(pinocchio REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)

  # The tests check the library on a real robot: the URDF is generated from the robot's xacro
  # at build time, and names/limits come from its robot.yaml.
  set(TEST_ROBOT panda)
  set(TEST_ROBOT_DIR "${arm_sandbox_description_DIR}/../${TEST_ROBOT}")
  set(TEST_URDF "${CMAKE_CURRENT_BINARY_DIR}/${TEST_ROBOT}.urdf")
  add_custom_command(
    OUTPUT ${TEST_URDF}
    COMMAND ${XACRO_EXECUTABLE} ${TEST_ROBOT_DIR}/urdf/${TEST_ROBOT}.urdf.xacro -o ${TEST_URDF}
    DEPENDS ${TEST_ROBOT_DIR}/urdf/${TEST_ROBOT}.urdf.xacro
    COMMENT "Generating ${TEST_ROBOT}.urdf for the kinematics tests")
  add_custom_target(test_urdf ALL DEPENDS ${TEST_URDF})

  foreach(test_name test_kinematic_chain test_ik)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    add_dependencies(${test_name} test_urdf)
    target_link_libraries(${test_name} ${PROJECT_NAME} pinocchio::pinocchio yaml-cpp)
    target_compile_definitions(${test_name} PRIVATE
      TEST_URDF_PATH="${TEST_URDF}"
      TEST_ROBOT_CONFIG_PATH="${TEST_ROBOT_DIR}/config/robot.yaml")
  endforeach()
endif()

ament_package()
