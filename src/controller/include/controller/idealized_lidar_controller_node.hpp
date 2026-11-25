#pragma once

#include "controller/controller_base.hpp"

// Teleports the model along the waypoint polyline via /world/<w>/set_pose.
// Dead-reckons: Gazebo's pose is only used to initialize the local frame.
class IdealizedLidarControllerNode : public controller::ControllerBase
{
public:
  IdealizedLidarControllerNode();

protected:
  bool step(const controller::StepContext & ctx) override;
  void onNewTarget() override {snap_pending_ = false;}

private:
  bool teleport(const controller::Waypoint & world_pos, double yaw);

  double speed_{0.5};               // [m/s]
  double max_yaw_rate_{0.5};        // [rad/s] (smooth mode only)
  double align_threshold_{0.035};   // [rad]
  bool snap_turn_{true};            // true: pause, jump yaw, pause. false: rate-limited turn
  double dwell_s_{0.5};             // [s] pause before and after a snap turn

  bool snap_pending_{false};
  double hold_until_{-1.0};
};
