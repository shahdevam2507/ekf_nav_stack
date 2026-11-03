#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>

// Teleop node for differential drive velocity control.
//
// Publishes geometry_msgs/Twist on /cmd_vel1:
//   linear.x  = vb  [m/s]   — body linear velocity
//   angular.z = w   [rad/s] — body angular velocity
//
// The micro-ROS firmware must convert (vb, w) → wheel velocities:
//   v_left  = vb - w * (L/2)
//   v_right = vb + w * (L/2)
// where L is the wheel track (axle width) in metres.

class teleop : public rclcpp::Node {
public:
    teleop() : Node("teleop_ps4")
    {
        // --- Robot physical parameters ---
        // wheel_radius [m]  — radius of each drive wheel
        wheel_radius_  = this->declare_parameter<double>("wheel_radius",  0.0367);
        // track_length [m]  — centre-to-centre distance between left and right wheels
        track_length_  = this->declare_parameter<double>("track_length",  0.239);

        // --- Velocity limits ---
        // max_linear_speed  [m/s]   — body forward speed limit
        max_linear_speed_  = this->declare_parameter<double>("max_linear_speed",  0.2305);
        // max_angular_speed [rad/s] — body yaw-rate limit
        //   physical upper bound: 2 * vb_max / L
        max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.929);
        // deadband — joystick axis values below this threshold are zeroed
        deadband_          = this->declare_parameter<double>("deadband", 0.05);

        RCLCPP_INFO(this->get_logger(),
            "Robot params — wheel_radius: %.4f m, track_length: %.4f m",
            wheel_radius_, track_length_);
        RCLCPP_INFO(this->get_logger(),
            "Speed limits  — max_vb: %.3f m/s, max_w: %.3f rad/s",
            max_linear_speed_, max_angular_speed_);

        sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&teleop::joy_callback, this, std::placeholders::_1));

        pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
    }

private:
    // Apply deadband: zero out small joystick noise
    double applyDeadband(double value) const
    {
        return (std::fabs(value) < deadband_) ? 0.0 : value;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() <= 3) {
            return;
        }

        // axes[1] = left stick Y  → vb  (forward/backward)
        // axes[3] = right stick X → w   (turn left/right, negated for ROS convention)
        double raw_lin = applyDeadband(msg->axes[1]);
        double raw_ang = applyDeadband(msg->axes[3]);

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x  =  max_linear_speed_  * raw_lin;   // vb  [m/s]
        cmd_vel.angular.z = max_angular_speed_  * raw_ang;   // w   [rad/s] make changes if the turn is inverted

        pub->publish(cmd_vel);
    }

    // Physical robot parameters
    double wheel_radius_;    // [m]
    double track_length_;    // [m]  (centre-to-centre axle width)

    // Velocity limits
    double max_linear_speed_;   // [m/s]
    double max_angular_speed_;  // [rad/s]
    double deadband_;

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