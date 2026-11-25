#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include "controller/controller_base.hpp"

// Differential-drive tracker: rotate in place, then drive along the leg.
// Reads Gazebo's pose every update (closed loop) and publishes /cmd_vel.
class ControllerNode : public controller::ControllerBase
{
public:
  ControllerNode();

protected:
  bool step(const controller::StepContext & ctx) override;
  void onNewTarget() override {state_ = State::ROTATE;}
  void onFinished() override {publishStop();}

private:
  enum class State { ROTATE, DRIVE };

  void publishStop();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  double angular_gain_{2.0};
  double max_angular_{5.0};
  double linear_gain_{0.4};
  double max_linear_{3.0};
  double heading_threshold_{5.0 * M_PI / 180.0};
  double drive_steer_gain_{0.0};     // 0 = no steering while driving (previous behaviour)

  State state_{State::ROTATE};
};
