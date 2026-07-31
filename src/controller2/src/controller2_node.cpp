#include "controller2/controller2_node.hpp"

#include <algorithm>
#include <cmath>

ControllerNode::ControllerNode()
: Node("controller_node"),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0),
  initialized_(false),
  origin_x_(0.0),
  origin_y_(0.0),
  origin_yaw_(0.0),
  finished_(false),
  current_waypoint_(0)
{
    waypoints_ =
    {
        {2.0, 0.0},
        {2.0,-0.5},
        {0.0, 0.0}
    };

    gz_node_.Subscribe(
        "/world/winding_room/dynamic_pose/info",
        &ControllerNode::poseCallback,
        this);

    cmd_vel_pub_ =
        create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10);

    control_timer_ =
        create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ControllerNode::controlLoop,this));

    RCLCPP_INFO(get_logger(),"Controller2 started.");
}

void ControllerNode::poseCallback(const gz::msgs::Pose_V &msg)
{
    for(int i=0;i<msg.pose_size();i++)
    {
        const auto &pose = msg.pose(i);

        if(pose.name()!="ddmr")
            continue;

        robot_x_=pose.position().x();
        robot_y_=pose.position().y();

        const auto &q=pose.orientation();

        robot_yaw_=std::atan2(
            2.0*(q.w()*q.z()+q.x()*q.y()),
            1.0-2.0*(q.y()*q.y()+q.z()*q.z()));

        if(!initialized_)
        {
            initialized_=true;

            origin_x_=robot_x_;
            origin_y_=robot_y_;
            origin_yaw_=robot_yaw_;

            RCLCPP_INFO(get_logger(),"Local frame initialized.");
        }

        break;
    }
}

void ControllerNode::controlLoop()
{
    if(!initialized_ || finished_)
        return;

    geometry_msgs::msg::Twist cmd;

    //----------------------------------
    // Local pose
    //----------------------------------

    double x = robot_x_ - origin_x_;
    double y = robot_y_ - origin_y_;
    double yaw = robot_yaw_ - origin_yaw_;

    //----------------------------------
    // Goal
    //----------------------------------

    const auto &goal = waypoints_[current_waypoint_];

    double dx = goal.x - x;
    double dy = goal.y - y;

    double rho = std::hypot(dx,dy);

    double alpha = std::atan2(dy,dx)-yaw;

    while(alpha>M_PI)
        alpha-=2*M_PI;

    while(alpha<-M_PI)
        alpha+=2*M_PI;

    //----------------------------------
    // Gains
    //----------------------------------

    constexpr double kv = 0.8;
    constexpr double kw = 3.0;

    double v = kv*rho;
    double w = kw*alpha;

    //----------------------------------
    // Slow down while turning
    //----------------------------------

    v *= std::cos(alpha);

    if(v<0.0)
        v=0.0;

    //----------------------------------
    // Clamp
    //----------------------------------

    cmd.linear.x =
        std::clamp(
            v,
            0.0,
            0.5);

    cmd.angular.z =
        std::clamp(
            w,
            -4.0,
            4.0);

    //----------------------------------
    // Waypoint reached
    //----------------------------------

    constexpr double goal_threshold = 0.08;

    if(rho < goal_threshold)
    {
        current_waypoint_++;

        if(current_waypoint_>=waypoints_.size())
        {
            finished_=true;

            cmd.linear.x=0.0;
            cmd.angular.z=0.0;

            RCLCPP_INFO(get_logger(),"Mission complete.");
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "Waypoint %zu reached.",
                current_waypoint_-1);
        }
    }

    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Pose %.2f %.2f Goal %.2f %.2f rho %.2f alpha %.2f",
        x,
        y,
        goal.x,
        goal.y,
        rho,
        alpha);
}