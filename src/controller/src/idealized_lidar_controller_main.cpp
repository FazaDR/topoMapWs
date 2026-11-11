#include "controller/idealized_lidar_controller_node.hpp"

#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IdealizedLidarControllerNode>());
  rclcpp::shutdown();
  return 0;
}
