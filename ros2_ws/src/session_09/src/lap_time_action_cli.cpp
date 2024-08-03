#include "lap_time_action_cli.hpp"


LapTimeActionClient::LapTimeActionClient()
: Node("lap_time_action_client"),
  lap_counter_(1) 
{
  this->client_ = rclcpp_action::create_client<MeasureLapTime>(
    this,
    "lap_timer");

  // Open the file to log lap results
  log_file_.open("lap_results.txt", std::ios::out | std::ios::app);

  if (!log_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open log file");
  }

  // Initial delay before sending the first goal
  this->initial_timer_ = this->create_wall_timer(
    10s, std::bind(&LapTimeActionClient::send_goal, this));
}

LapTimeActionClient::~LapTimeActionClient()
{
  // Close the log file when the node is destroyed
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

void LapTimeActionClient::send_goal()
{
  this->initial_timer_->cancel();
  if (!client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    return;
  }

  auto goal_msg = MeasureLapTime::Goal();

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<MeasureLapTime>::SendGoalOptions();

  send_goal_options.goal_response_callback = std::bind(
    &LapTimeActionClient::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &LapTimeActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(
    &LapTimeActionClient::result_callback, this, std::placeholders::_1);

  this->client_->async_send_goal(goal_msg, send_goal_options);
}

void LapTimeActionClient::goal_response_callback(
  const rclcpp_action::ClientGoalHandle<MeasureLapTime>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Goal accepted");
}

void LapTimeActionClient::feedback_callback(
  GoalHandleMeasureLapTime::SharedPtr,
  const std::shared_ptr<const MeasureLapTime::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback received: elapsed time: %f", feedback->elapsed_time);
}

void LapTimeActionClient::result_callback(const GoalHandleMeasureLapTime::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Result received: total time: %f", result.result->total_time);
      // Log the result to the file
      if (log_file_.is_open()) {
        log_file_ << "Lap " << lap_counter_++ << ": " << result.result->total_time << " s\n";
      }
      // Resend goal after receiving result
      send_goal();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      RCLCPP_WARN(this->get_logger(), "Result is unknown");
      break;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LapTimeActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
