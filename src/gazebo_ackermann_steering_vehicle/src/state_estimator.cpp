#include "state_estimator.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace path_follower {

StateEstimator::StateEstimator(rclcpp::Node* node)
    : node_(node) {
    // Create TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(node_->get_logger(), "StateEstimator initialized with frames: %s -> %s",
                world_frame_.c_str(), base_frame_.c_str());
}

bool StateEstimator::getState(VehicleState& state) {
    try {
        // Get transform from world to base_link
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);

        // Extract position
        state.x = transform_stamped.transform.translation.x;
        state.y = transform_stamped.transform.translation.y;

        // Extract yaw from quaternion
        state.theta = quaternionToYaw(transform_stamped.transform.rotation);

        // Get timestamp
        state.timestamp = transform_stamped.header.stamp.sec +
                         transform_stamped.header.stamp.nanosec * 1e-9;

        // Estimate velocity via numerical differentiation
        if (has_prev_state_) {
            double dt = state.timestamp - prev_state_.timestamp;
            if (dt > 0.0) {
                double dx = state.x - prev_state_.x;
                double dy = state.y - prev_state_.y;
                state.v = std::sqrt(dx * dx + dy * dy) / dt;
            } else {
                state.v = prev_state_.v;
            }
        } else {
            state.v = 0.0;
        }

        // Update previous state
        prev_state_ = state;
        has_prev_state_ = true;

        return true;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Could not get transform: %s", ex.what());
        return false;
    }
}

void StateEstimator::reset() {
    has_prev_state_ = false;
    prev_state_ = VehicleState();
}

double StateEstimator::quaternionToYaw(const geometry_msgs::msg::Quaternion& quat) const {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quat, tf_quat);

    // Convert to RPY
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    return yaw;
}

} // namespace path_follower
