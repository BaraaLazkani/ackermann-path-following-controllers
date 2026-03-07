#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "path_follower_base.hpp"

namespace path_follower {

/**
 * @brief Estimates vehicle state using TF transforms
 *
 * Extracts ground-truth pose from Gazebo via TF tree and computes
 * velocity via numerical differentiation.
 */
class StateEstimator {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node pointer for creating TF listener
     */
    explicit StateEstimator(rclcpp::Node* node);

    /**
     * @brief Get current vehicle state from TF
     * @param state Output vehicle state
     * @return true if state was successfully retrieved
     */
    bool getState(VehicleState& state);

    /**
     * @brief Reset internal state
     */
    void reset();

private:
    rclcpp::Node* node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Frame names
    std::string world_frame_{"odom"};
    std::string base_frame_{"body_link"};

    // Previous state for velocity estimation
    VehicleState prev_state_;
    bool has_prev_state_{false};

    /**
     * @brief Extract yaw angle from quaternion
     */
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& quat) const;
};

} // namespace path_follower

#endif // STATE_ESTIMATOR_HPP
