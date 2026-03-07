#include "pid_controller.hpp"
#include <cmath>

namespace path_follower {

PIDController::PIDController(rclcpp::Node* node) : node_(node) {
    // Declare and get PID parameters
    node_->declare_parameter("pid.kp", kp_);
    node_->declare_parameter("pid.ki", ki_);
    node_->declare_parameter("pid.kd", kd_);
    node_->declare_parameter("pid.base_velocity", base_velocity_);
    node_->declare_parameter("pid.curvature_gain", curvature_gain_);
    node_->declare_parameter("pid.max_integral", max_integral_);

    kp_ = node_->get_parameter("pid.kp").as_double();
    ki_ = node_->get_parameter("pid.ki").as_double();
    kd_ = node_->get_parameter("pid.kd").as_double();
    base_velocity_ = node_->get_parameter("pid.base_velocity").as_double();
    curvature_gain_ = node_->get_parameter("pid.curvature_gain").as_double();
    max_integral_ = node_->get_parameter("pid.max_integral").as_double();

    RCLCPP_INFO(node_->get_logger(), "PID Controller Parameters:");
    RCLCPP_INFO(node_->get_logger(), "  Kp: %.3f, Ki: %.3f, Kd: %.3f", kp_, ki_, kd_);
    RCLCPP_INFO(node_->get_logger(), "  Base velocity: %.2f m/s", base_velocity_);
    RCLCPP_INFO(node_->get_logger(), "  Curvature gain: %.2f", curvature_gain_);
}

ControlCommand PIDController::computeControl(
    const VehicleState& current_state,
    const ReferenceState& reference_state,
    double cross_track_error) {

    ControlCommand cmd;

    // === Lateral Control (PID on CTE) ===
    double error = cross_track_error;

    // Initialize on first call
    if (first_call_) {
        prev_error_ = error;
        prev_time_ = current_state.timestamp;
        first_call_ = false;
    }

    // Compute time step
    double dt = current_state.timestamp - prev_time_;
    if (dt <= 0.0) {
        dt = 0.02;  // Default 50Hz
    }

    // Proportional term
    double p_term = kp_ * error;

    // Integral term with anti-windup
    integral_error_ += error * dt;
    integral_error_ = clamp(integral_error_, -max_integral_, max_integral_);
    double i_term = ki_ * integral_error_;

    // Derivative term
    double error_rate = (error - prev_error_) / dt;
    double d_term = kd_ * error_rate;

    // Compute steering angle
    double steering = p_term + i_term + d_term;

    // Add heading error correction
    double heading_error = normalizeAngle(reference_state.theta - current_state.theta);
    steering += 0.5 * heading_error;  // Simple heading correction

    // Clamp steering to limits
    cmd.steering_angle = clamp(steering, -max_steering_angle_, max_steering_angle_);

    // === Longitudinal Control (Velocity based on curvature) ===
    // Slow down for high curvature sections
    double curvature = reference_state.kappa;
    double velocity = base_velocity_ * std::exp(-curvature_gain_ * std::abs(curvature));

    // CRITICAL: Ensure minimum velocity to avoid getting stuck on tight curves
    double min_velocity = 0.3;  // Never go below 0.3 m/s
    velocity = std::max(velocity, min_velocity);

    // Use reference velocity if available and reasonable
    if (reference_state.v_ref > 0.0) {
        velocity = std::min(velocity, reference_state.v_ref);
    }

    // Clamp velocity to limits
    cmd.velocity = clamp(velocity, 0.0, max_velocity_);

    // Update state for next iteration
    prev_error_ = error;
    prev_time_ = current_state.timestamp;

    return cmd;
}

void PIDController::reset() {
    integral_error_ = 0.0;
    prev_error_ = 0.0;
    prev_time_ = 0.0;
    first_call_ = true;
}

} // namespace path_follower
