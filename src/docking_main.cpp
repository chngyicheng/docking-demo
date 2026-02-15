#include "docking_demo/docking_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<docking_demo::DockingNode>());
  rclcpp::shutdown();
  return 0;
}
