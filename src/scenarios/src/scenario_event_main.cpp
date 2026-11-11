#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

class ScenarioEventNode final : public rclcpp::Node
{
public:
  ScenarioEventNode()
  : Node("scenario_event_node"),
    world_(declare_parameter("world", "divider_room")),
    model_(declare_parameter("model", "disappearing_wall")),
    remove_after_sec_(declare_parameter("remove_after_sec", 30.0))
  {
    if (remove_after_sec_ <= 0.0) {
      throw std::invalid_argument("remove_after_sec must be positive");
    }

    timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(remove_after_sec_)),
      std::bind(&ScenarioEventNode::removeModel, this));

    RCLCPP_INFO(
      get_logger(),
      "Scheduled removal of '%s' after %.1f simulation seconds.",
      model_.c_str(), remove_after_sec_);
  }

private:
  void removeModel()
  {
    timer_->cancel();

    gz::msgs::Entity request;
    request.set_name(model_);
    request.set_type(gz::msgs::Entity::MODEL);

    gz::msgs::Boolean reply;
    bool result = false;
    const bool executed = gz_node_.Request(
      "/world/" + world_ + "/remove",
      request,
      1000,
      reply,
      result);

    if (!executed || !result || !reply.data()) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to remove scenario model '%s' from world '%s'.",
        model_.c_str(), world_.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Removed scenario model '%s' at simulation time %.3f seconds.",
      model_.c_str(), get_clock()->now().seconds());
  }

  std::string world_;
  std::string model_;
  double remove_after_sec_;
  gz::transport::Node gz_node_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScenarioEventNode>());
  rclcpp::shutdown();
  return 0;
}
