cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS-free Python module (scene composition), shared by the launch files and, later, the Gym env.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scene test/test_scene.py)
endif()

ament_package()
