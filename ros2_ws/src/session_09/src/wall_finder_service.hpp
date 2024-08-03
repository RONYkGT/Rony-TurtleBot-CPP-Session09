#ifndef WALL_FINDER_SERVICE_HPP
#define WALL_FINDER_SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "turtle_controller_interfaces/srv/find_closest_wall.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WallFinderService : public rclcpp::Node
{
public:
  WallFinderService();

private:
  void handle_request(
    const std::shared_ptr<turtle_controller_interfaces::srv::FindClosestWall::Request> request,
    std::shared_ptr<turtle_controller_interfaces::srv::FindClosestWall::Response> response);

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Service<turtle_controller_interfaces::srv::FindClosestWall>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  bool received_request_;
  bool finished_;
};

#endif // WALL_FINDER_SERVICE_HPP
