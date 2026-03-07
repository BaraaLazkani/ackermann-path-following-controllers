#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "path_follower_base.hpp"

namespace path_follower {

/**
 * @brief PID controller for path following
 *
 * Lateral control: PID on cross-track error -> steering angle
 * Longitudinal control: Velocity based on path curvature
 */
class PIDController : public PathFollowerBase {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for parameter retrieval
     */
    explicit PIDController(rclcpp::Node* node);

    /**
     * @brief Compute control commands
     */
    ControlCommand computeControl(
        const VehicleState& current_state,
        const ReferenceState& reference_state,
        double cross_track_error) override;

    /**
     * @brief Reset controller state
     */
    void reset() override;

    /**
     * @brief Get controller name
     */
    std::string getName() const override { return "PID"; }

private:
    // PID gains for lateral control
    double kp_{1.5};      // Proportional gain
    double ki_{0.05};     // Integral gain
    double kd_{0.3};      // Derivative gain

    // Velocity control parameters
    double base_velocity_{1.0};     // Base velocity [m/s]
    double curvature_gain_{2.0};    // Curvature scaling factor

    // PID state
    double integral_error_{0.0};
    double prev_error_{0.0};
    double prev_time_{0.0};
    bool first_call_{true};

    // Anti-windup
    double max_integral_{1.0};  // Maximum integral term

    rclcpp::Node* node_;
};

} // namespace path_follower

#endif // PID_CONTROLLER_HPP
