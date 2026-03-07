#include "stanley_controller.hpp"
#include <cmath>

namespace path_follower {

StanleyController::StanleyController(rclcpp::Node* node) : node_(node) {
    // Declare and get Stanley parameters
    node_->declare_parameter("stanley.k", k_);
    node_->declare_parameter("stanley.k_s", k_s_);
    node_->declare_parameter("stanley.base_velocity", base_velocity_);
    node_->declare_parameter("stanley.curvature_gain", curvature_gain_);

    k_ = node_->get_parameter("stanley.k").as_double();
    k_s_ = node_->get_parameter("stanley.k_s").as_double();
    base_velocity_ = node_->get_parameter("stanley.base_velocity").as_double();
    curvature_gain_ = node_->get_parameter("stanley.curvature_gain").as_double();

    RCLCPP_INFO(node_->get_logger(), "Stanley Controller Parameters:");
    RCLCPP_INFO(node_->get_logger(), "  k: %.3f, k_s: %.3f", k_, k_s_);
    RCLCPP_INFO(node_->get_logger(), "  Base velocity: %.2f m/s", base_velocity_);
    RCLCPP_INFO(node_->get_logger(), "  Curvature gain: %.2f", curvature_gain_);
}

ControlCommand StanleyController::computeControl(
    const VehicleState& current_state,
    const ReferenceState& reference_state,
    double cross_track_error) {

    ControlCommand cmd;

    // === Stanley Lateral Control ===

    // 1. Heading error component
    double heading_error = normalizeAngle(reference_state.theta - current_state.theta);

    // 2. Cross-track error component (velocity-adaptive)
    // Use commanded velocity if actual velocity is too low (startup condition)
    double velocity = current_state.v;
    if (velocity < 0.1) {
        velocity = base_velocity_;  // Use base velocity during startup
    }
    double cte_term = std::atan(k_ * cross_track_error / (k_s_ + velocity));

    // 3. Stanley control law
    double steering = heading_error + cte_term;

    // Clamp steering to limits
    cmd.steering_angle = clamp(steering, -max_steering_angle_, max_steering_angle_);

    // === Longitudinal Control (Velocity based on curvature) ===
    double curvature = reference_state.kappa;
    double target_velocity = base_velocity_ * std::exp(-curvature_gain_ * std::abs(curvature));

    // CRITICAL: Ensure minimum velocity to avoid getting stuck on tight curves
    double min_velocity = 0.3;  // Never go below 0.3 m/s
    target_velocity = std::max(target_velocity, min_velocity);

    // Use reference velocity if available
    if (reference_state.v_ref > 0.0) {
        target_velocity = std::min(target_velocity, reference_state.v_ref);
    }

    // Clamp velocity to limits
    cmd.velocity = clamp(target_velocity, 0.0, max_velocity_);

    return cmd;
}

void StanleyController::reset() {
    // Stanley controller is stateless, nothing to reset
}

} // namespace path_follower
