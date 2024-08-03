#include "wall_finder_service.hpp"

WallFinderService::WallFinderService()
: Node("wall_finder_service"), received_request_(false), finished_(false)
{
  service_ = this->create_service<turtle_controller_interfaces::srv::FindClosestWall>(
    "find_closest_wall", std::bind(&WallFinderService::handle_request, this, std::placeholders::_1, std::placeholders::_2));
  
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&WallFinderService::scan_callback, this, std::placeholders::_1));
  
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void WallFinderService::handle_request(
  const std::shared_ptr<turtle_controller_interfaces::srv::FindClosestWall::Request> request,
  std::shared_ptr<turtle_controller_interfaces::srv::FindClosestWall::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Incoming request");
  received_request_ = true;
  response->success = true;
}

void WallFinderService::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!received_request_) return;

  if(msg->ranges[0] < 0.5){
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    publisher_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "Reached closest wall.");
    finished_ = true;
    received_request_ = false;
    return;
  }
  
  // Initialize minimum distance and angle
  float min_distance = msg->range_max;
  int min_index = -1;

  // Find the closest distance and its corresponding index
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < min_distance) {
      min_distance = msg->ranges[i];
      min_index = i;
    }
  }

  auto twist = geometry_msgs::msg::Twist();
  if(std::abs(msg->ranges[0] - min_distance) >= 0.1)
  {
    RCLCPP_INFO(this->get_logger(), "Turning");
    twist.linear.x = 0.0;
    if(min_index <= 0 && min_index > 90)
    {
      twist.angular.z = -0.5;
    }
    else
    {
      twist.angular.z = 0.5;
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Moving forward");
    twist.linear.x = 0.5;
    twist.angular.z = 0.0;
  }

  publisher_->publish(twist);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFinderService>());
  rclcpp::shutdown();
  return 0;
}
