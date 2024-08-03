#ifndef LAP_TIME_ACTION_SRV_HPP
#define LAP_TIME_ACTION_SRV_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtle_controller_interfaces/action/measure_lap_time.hpp"

using namespace std::chrono_literals;
using MeasureLapTime = turtle_controller_interfaces::action::MeasureLapTime;
using GoalHandleLapTime = rclcpp_action::ServerGoalHandle<MeasureLapTime>;

class LapTimeActionServer : public rclcpp::Node
{
public:
  LapTimeActionServer();

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MeasureLapTime::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLapTime> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleLapTime> goal_handle);

  void execute(const std::shared_ptr<GoalHandleLapTime> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp_action::Server<MeasureLapTime>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  double start_time_, current_time_;
  bool lap_completed_, start_timer_;
  double start_x_;
  double start_y_;
};

#endif // LAP_TIME_ACTION_SERVER_HPP
