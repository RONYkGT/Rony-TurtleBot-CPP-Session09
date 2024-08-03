#ifndef ROBOT_DRIVER_HPP
#define ROBOT_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtle_controller_interfaces/srv/find_closest_wall.hpp"
#include <chrono>

class RobotDriver : public rclcpp::Node
{
public:
    RobotDriver();

private:
    void response_callback(rclcpp::Client<turtle_controller_interfaces::srv::FindClosestWall>::SharedFuture future);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<turtle_controller_interfaces::srv::FindClosestWall>::SharedPtr client_;
    bool at_closest_wall_;
};

#endif // ROBOT_DRIVER_HPP
