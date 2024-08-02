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
  LapTimeActionServer()
  : Node("lap_time_action_server")
  {
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<MeasureLapTime>(
      this,
      "lap_timer",
      std::bind(&LapTimeActionServer::handle_goal, this, _1, _2),
      std::bind(&LapTimeActionServer::handle_cancel, this, _1),
      std::bind(&LapTimeActionServer::handle_accepted, this, _1));

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&LapTimeActionServer::odom_callback, this, _1));

    start_time_ = 0.0;
    lap_completed_ = false;
    start_x_ = 0.0;
    start_y_ = 0.0;
    start_timer_ = false;
    current_time_ = 0.0;
  }

private:
  rclcpp_action::Server<MeasureLapTime>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  double start_time_, current_time_;
  bool lap_completed_, start_timer_;
  double start_x_;
  double start_y_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MeasureLapTime::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid; 
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLapTime> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleLapTime> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    std::thread{std::bind(&LapTimeActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleLapTime> goal_handle)
  {
    
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    auto result = std::make_shared<MeasureLapTime::Result>();
    lap_completed_ = false;
    start_timer_ = true;
    while(start_time_ == 0) continue;
    while (!lap_completed_) {
      if (goal_handle->is_canceling()) {
        result->total_time = 0.0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      auto feedback = std::make_shared<MeasureLapTime::Feedback>();
      current_time_ = this->now().seconds() - start_time_;
      feedback->elapsed_time = current_time_;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    start_timer_ = false;
    start_time_ = 0.0;
    result->total_time = current_time_;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!start_timer_) return;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    RCLCPP_INFO(this->get_logger(), "starting x: %f, y: %f - x: %f, y: %f, start time: %f, current time: %f, xdiff: %f, ydiff: %f",start_x_, start_y_, x, y, start_time_, current_time_, std::abs(x-start_x_), std::abs(y - start_y_));
    if (start_time_ == 0.0) {
      start_time_ = this->now().seconds();
      start_x_ = x;
      start_y_ = y;
      RCLCPP_INFO(this->get_logger(), "Starting position x: %f, y: %f", start_x_, start_y_ );
    }
    else if (std::abs(x-start_x_) < 0.15 && std::abs(y - start_y_) < 0.15 && current_time_ > 3) {
      lap_completed_ = true;
      RCLCPP_INFO(this->get_logger(), "Lap completed");
    }

    
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LapTimeActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
