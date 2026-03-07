#ifndef STANLEY_CONTROLLER_HPP
#define STANLEY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "path_follower_base.hpp"

namespace path_follower {

/**
 * @brief Stanley controller for path following
 *
 * Control law: δ = θ_e + atan(k·e / (k_s + v))
 * Combines heading error with velocity-adaptive cross-track correction
 */
class StanleyController : public PathFollowerBase {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for parameter retrieval
     */
    explicit StanleyController(rclcpp::Node* node);

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
    std::string getName() const override { return "Stanley"; }

private:
    // Stanley control gains
    double k_{2.5};       // Cross-track error gain
    double k_s_{1.0};     // Softening constant for velocity adaptation

    // Velocity control parameters
    double base_velocity_{1.0};     // Base velocity [m/s]
    double curvature_gain_{2.0};    // Curvature scaling factor

    rclcpp::Node* node_;
};

} // namespace path_follower

#endif // STANLEY_CONTROLLER_HPP
