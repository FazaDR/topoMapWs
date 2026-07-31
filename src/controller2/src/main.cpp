#include "controller2/controller2_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);

    auto node =
        std::make_shared<ControllerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}