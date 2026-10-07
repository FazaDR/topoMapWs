#include "mapping_mlatc/mlatc_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MlatcNode>());
  rclcpp::shutdown();
  return 0;
}
