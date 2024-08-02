#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotDriver : public rclcpp::Node
{
public:
    RobotDriver()
    : Node("driver")
    {
        RCLCPP_INFO(this->get_logger(), "robot initialized");
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&RobotDriver::scan_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        if(msg->ranges[0] < 0.5)
        {
            RCLCPP_INFO(this->get_logger(), "turning right");
            twist.linear.x = 0.0;
            twist.angular.z = -1.0;
        }
        else if(msg->ranges[20] < 0.3)
        {
            RCLCPP_INFO(this->get_logger(), "turning right slower");
            twist.linear.x = 0.0;
            twist.angular.z = -0.25;
        }
        else 
        {
            twist.linear.x = 0.5;
            twist.angular.z = 0.0;
        }
        publisher_->publish(twist);
        
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
