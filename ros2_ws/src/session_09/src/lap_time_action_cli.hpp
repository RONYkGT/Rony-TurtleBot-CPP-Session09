#ifndef LAP_TIME_ACTION_CLI_HPP
#define LAP_TIME_ACTION_CLI_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "turtle_controller_interfaces/action/measure_lap_time.hpp"
#include <fstream>  

using namespace std::chrono_literals;
using MeasureLapTime = turtle_controller_interfaces::action::MeasureLapTime;
using GoalHandleMeasureLapTime = rclcpp_action::ClientGoalHandle<MeasureLapTime>;

class LapTimeActionClient : public rclcpp::Node
{
public:
  LapTimeActionClient();
  ~LapTimeActionClient();  // Destructor to handle file cleanup

private:
  rclcpp_action::Client<MeasureLapTime>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr initial_timer_;
  std::ofstream log_file_;  // For logging lap results
  int lap_counter_;  // To keep track of lap numbers

  void send_goal();
  void goal_response_callback(
    const rclcpp_action::ClientGoalHandle<MeasureLapTime>::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleMeasureLapTime::SharedPtr,
    const std::shared_ptr<const MeasureLapTime::Feedback> feedback);
  void result_callback(const GoalHandleMeasureLapTime::WrappedResult & result);
};

#endif  // LAP_TIME_ACTION_CLI_HPP
