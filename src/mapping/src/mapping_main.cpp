#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mapping/mapping_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}