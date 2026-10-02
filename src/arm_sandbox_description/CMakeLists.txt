cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_description)

find_package(ament_cmake REQUIRED)

# One folder per robot (FR-7). Installed as share/arm_sandbox_description/<robot>/.
install(DIRECTORY panda DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_panda_mjcf test/test_panda_mjcf.py)
endif()

ament_package()
