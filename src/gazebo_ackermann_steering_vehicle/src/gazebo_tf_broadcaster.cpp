#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class GazeboTFBroadcaster : public rclcpp::Node {
public:
    GazeboTFBroadcaster() : Node("gazebo_tf_broadcaster") {
        // Create TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscribe to odometry from Gazebo
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/ackermann_steering_vehicle/odometry",
            10,
            std::bind(&GazeboTFBroadcaster::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Gazebo TF Broadcaster started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /model/ackermann_steering_vehicle/odometry");
        RCLCPP_INFO(this->get_logger(), "Publishing TF: odom -> body_link");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "body_link";

        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        transform.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);

        static bool first_msg = true;
        if (first_msg) {
            RCLCPP_INFO(this->get_logger(), "Publishing TF transforms!");
            first_msg = false;
        }
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
