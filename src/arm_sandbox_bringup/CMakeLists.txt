cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/activate_at_home DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Each launch test gets its own ROS domain, so sims started by tests of different packages
  # (colcon runs packages in parallel) or a `make sim` in another terminal can't interfere.
  add_launch_test(test/test_sim_bringup.py TIMEOUT 180 ENV ROS_DOMAIN_ID=41)
  add_launch_test(test/test_arm_trajectory.py TIMEOUT 300 ENV ROS_DOMAIN_ID=42)
  add_launch_test(test/test_sim_recording.py TIMEOUT 180 ENV ROS_DOMAIN_ID=44)
endif()

ament_package()
