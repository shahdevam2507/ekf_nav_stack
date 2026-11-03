#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
class teleop : public rclcpp::Node{
    public:
    teleop() : Node("teleop_ps4")
    {
        sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&teleop::joy_callback, this, std::placeholders::_1));
        pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
    private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = -msg->axes[1];
        cmd_vel.angular.z = msg->axes[3];
        pub->publish(cmd_vel);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<teleop>());
    rclcpp::shutdown();
    return 0;
}