#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class TrackerNode : public rclcpp::Node {
public:
    TrackerNode() : Node("tracker_node") {
        // Create a subscription to the path topic using nav_msgs::msg::Path message type
        path_subscription_ = create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&TrackerNode::pathCallback, this, std::placeholders::_1));
        
        // create subscriber for current position of type odom
        odom_sub = create_subscription<nav_msgs::msg::Odometry>("/wamv/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_position_.x = msg->pose.pose.position.x;
            current_position_.y = msg->pose.pose.position.y;
        });

        // Create publisher for thrust and angle
        thrust_pub = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/main/thrust", 10);
        angle_pub = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/main/pos", 10);
        previous_error_thrust = 0;
        previous_error_angle = 0;
    }

    void pathCallback(const nav_msgs::msg::Path path_msg) {
        // Check if the path has at least 10 points
        try {
            // Get the 10th point from the path
            auto target_point = path_msg.poses[9];

            // Calculate the error between the current position and the target point
            double error_x = target_point.pose.position.x - current_position_.x;
            double error_y = target_point.pose.position.y - current_position_.y;

            // Calculate the control signal using PD controller
            auto distance = sqrt(pow(error_x, 2) + pow(error_y, 2));
            auto control_thrust = kp_thrust_ * distance + kd_thrust * (distance - previous_error_thrust);
            previous_error_thrust = distance;

            // Calculate the angle of the control signal
            auto error_angle = atan2(error_y, error_x);
            auto control_angle = kp_angle_ * error_angle + kd_angle * (error_angle - previous_error_angle);
            previous_error_angle = error_angle;


            //publish the control signal
            std_msgs::msg::Float64 thrust_msg;
            thrust_msg.data = control_thrust;
            thrust_pub->publish(thrust_msg);

            //publish the angle
            std_msgs::msg::Float64 angle_msg;
            angle_msg.data = control_angle;
            angle_pub->publish(angle_msg);

        }
        catch (const std::out_of_range& e) {
            RCLCPP_ERROR(this->get_logger(), "Path has less than 10 points");
        }
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub;

    geometry_msgs::msg::Point current_position_;
    double previous_error_x_ = 0.0;
    double previous_error_y_ = 0.0;
    double kp_thrust_ = 1.0;  // Proportional gain for thrust
    double kd_thrust = 0.1;  // Derivative gain for thrust
    double kp_angle_ = 1.0;  // Proportional gain for angle
    double kd_angle = 0.1;  // Derivative gain for angle;
    double previous_error_thrust;
    double previous_error_angle;
};

int main(int argc, char** argv) {
    // Initialize the ROS node
    rclcpp::init(argc, argv);

    // Create an instance of the TrackerNode class
    auto tracker_node = std::make_shared<TrackerNode>();

    // Spin the node
    rclcpp::spin(tracker_node);

    rclcpp::shutdown();

    return 0;
}