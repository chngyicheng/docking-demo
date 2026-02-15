#include "docking_demo/disturbance_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<docking_demo::DisturbanceNode>());
  rclcpp::shutdown();
  return 0;
}
