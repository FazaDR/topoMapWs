#include "mapping/atcdt_free.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AtcdtFreeNode>());
    rclcpp::shutdown();
    return 0;
}
