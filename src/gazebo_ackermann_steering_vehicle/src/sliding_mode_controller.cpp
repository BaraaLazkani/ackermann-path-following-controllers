#include "sliding_mode_controller.hpp"
#include <cmath>

namespace path_follower {

SlidingModeController::SlidingModeController(rclcpp::Node* node) : node_(node) {
    // Declare and get sliding mode parameters
    node_->declare_parameter("smc.lambda", lambda_);
    node_->declare_parameter("smc.k", k_);
    node_->declare_parameter("smc.eta", eta_);
    node_->declare_parameter("smc.epsilon", epsilon_);
    node_->declare_parameter("smc.base_velocity", base_velocity_);
    node_->declare_parameter("smc.curvature_gain", curvature_gain_);

    lambda_ = node_->get_parameter("smc.lambda").as_double();
    k_ = node_->get_parameter("smc.k").as_double();
    eta_ = node_->get_parameter("smc.eta").as_double();
    epsilon_ = node_->get_parameter("smc.epsilon").as_double();
    base_velocity_ = node_->get_parameter("smc.base_velocity").as_double();
    curvature_gain_ = node_->get_parameter("smc.curvature_gain").as_double();

    RCLCPP_INFO(node_->get_logger(), "Sliding Mode Controller Parameters:");
    RCLCPP_INFO(node_->get_logger(), "  λ: %.3f, k: %.3f, η: %.3f, ε: %.3f",
               lambda_, k_, eta_, epsilon_);
    RCLCPP_INFO(node_->get_logger(), "  Base velocity: %.2f m/s", base_velocity_);
}

ControlCommand SlidingModeController::computeControl(
    const VehicleState& current_state,
    const ReferenceState& reference_state,
    double cross_track_error) {

    ControlCommand cmd;

    // Initialize on first call
    if (first_call_) {
        prev_cte_ = cross_track_error;
        prev_time_ = current_state.timestamp;
        first_call_ = false;
    }

    // Compute time step
    double dt = current_state.timestamp - prev_time_;
    if (dt <= 0.0) {
        dt = 0.02;  // Default 50Hz
    }

    // === Sliding Mode Control ===

    // Cross-track error
    double e = cross_track_error;

    // CTE rate (numerical derivative)
    double e_dot = (e - prev_cte_) / dt;

    // Heading error
    double theta_e = normalizeAngle(reference_state.theta - current_state.theta);

    // Alternative SMC formulation: Separate heading and position control
    // Similar to Stanley but with SMC robustness for heading error

    // Heading error sliding surface with SMC
    double s_heading = theta_e;
    double heading_control = k_ * s_heading + eta_ * smoothSign(s_heading, epsilon_);

    // Position error term (similar to Stanley's CTE term)
    double velocity = std::max(base_velocity_, 0.1);
    double cte_control = std::atan(lambda_ * e / velocity);

    // Combined control law
    double steering = heading_control + cte_control;

    // Add feedforward for path curvature
    double kappa_ref = reference_state.kappa;
    double feedforward = std::atan(wheelbase_ * kappa_ref);
    steering += feedforward;

    // Clamp steering to limits
    cmd.steering_angle = clamp(steering, -max_steering_angle_, max_steering_angle_);

    // === Longitudinal Control ===
    double curvature = reference_state.kappa;
    double target_velocity = base_velocity_ * std::exp(-curvature_gain_ * std::abs(curvature));

    // CRITICAL: Ensure minimum velocity to avoid getting stuck on tight curves
    double min_velocity = 0.3;  // Never go below 0.3 m/s
    target_velocity = std::max(target_velocity, min_velocity);

    if (reference_state.v_ref > 0.0) {
        target_velocity = std::min(target_velocity, reference_state.v_ref);
    }

    cmd.velocity = clamp(target_velocity, 0.0, max_velocity_);

    // Update state for next iteration
    prev_cte_ = e;
    prev_time_ = current_state.timestamp;

    return cmd;
}

void SlidingModeController::reset() {
    prev_cte_ = 0.0;
    prev_time_ = 0.0;
    first_call_ = true;
}

} // namespace path_follower
