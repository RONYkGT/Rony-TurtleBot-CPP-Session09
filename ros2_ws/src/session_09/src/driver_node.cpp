#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtle_controller_interfaces/srv/find_closest_wall.hpp"
#include <chrono>
using namespace std::chrono_literals;
class RobotDriver : public rclcpp::Node
{
public:
    RobotDriver()
    : Node("driver")
    {
        client_ = this->create_client<turtle_controller_interfaces::srv::FindClosestWall>("find_closest_wall");
        
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the wall finding service to be available...");
        }
        auto request = std::make_shared<turtle_controller_interfaces::srv::FindClosestWall::Request>();
        auto result = client_->async_send_request(
         request,
         std::bind(
            &RobotDriver::response_callback, 
            this, 
            std::placeholders::_1
            )
        );

        RCLCPP_INFO(this->get_logger(), "robot initialized");
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&RobotDriver::scan_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        at_closest_wall_ = false;
        
    }

private:

    void response_callback(rclcpp::Client<turtle_controller_interfaces::srv::FindClosestWall>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Robot reached the wall. Starting behavior...");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach the wall. Continue normally");
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if(!at_closest_wall_ && msg->ranges[0] < 0.55)
        {
            at_closest_wall_ = true;
            RCLCPP_INFO(this->get_logger(), "Robot is at the wall");
        }
        else if(!at_closest_wall_) return;

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
        else if(msg->ranges[90] > 0.3 && msg->ranges[90] < 1.0)
        {
            RCLCPP_INFO(this->get_logger(), "turning left");
            twist.linear.x = 0.5;
            twist.angular.z = 0.3;
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
    rclcpp::Client<turtle_controller_interfaces::srv::FindClosestWall>::SharedPtr client_;
    
    bool at_closest_wall_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
