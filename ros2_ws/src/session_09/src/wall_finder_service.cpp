#include "rclcpp/rclcpp.hpp"
#include "turtle_controller_interfaces/srv/find_closest_wall.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

class WallFinderService : public rclcpp::Node
{
public:
  WallFinderService()
  : Node("wall_finder_service")
  {
    service_ = this->create_service<turtle_controller_interfaces::srv::FindClosestWall>(
        "find_closest_wall", std::bind(&WallFinderService::handle_request, this, _1, _2));
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&WallFinderService::scan_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    received_request_ = false;
    finished_ = false;
  }

private:
  void handle_request(
    const std::shared_ptr<turtle_controller_interfaces::srv::FindClosestWall::Request> request,
    std::shared_ptr<turtle_controller_interfaces::srv::FindClosestWall::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Incoming request");
    received_request_ = true;

    response->success = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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
    if(abs(msg->ranges[0] - min_distance) >= 0.075)
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
    else{
      RCLCPP_INFO(this->get_logger(), "Moving forward");
      twist.linear.x = 0.5;
      twist.angular.z = 0.0;

    }
    publisher_->publish(twist);
}


  rclcpp::Service<turtle_controller_interfaces::srv::FindClosestWall>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  bool received_request_, finished_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFinderService>());
  rclcpp::shutdown();
  return 0;
}
