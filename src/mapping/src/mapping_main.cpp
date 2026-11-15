#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mapping/mapping_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingNode>();
  rclcpp::spin(node);
  const int code = node->exit_code();
  node.reset();            // destructor saves the map
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return code;
}