#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "arm_sandbox_viz/rerun_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // The node is destroyed (and the recording flushed) before shutdown.
  rclcpp::spin(std::make_shared<arm_sandbox_viz::RerunBridge>());
  rclcpp::shutdown();
  return 0;
}
