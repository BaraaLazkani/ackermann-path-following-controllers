#ifndef SLIDING_MODE_CONTROLLER_HPP
#define SLIDING_MODE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "path_follower_base.hpp"

namespace path_follower {

/**
 * @brief Sliding Mode Controller for path following
 *
 * Sliding surface: s = θ_e + λ·e
 * Reaching law: ṡ = -k·s - η·sign(s)
 * Uses boundary layer to reduce chattering
 */
class SlidingModeController : public PathFollowerBase {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for parameter retrieval
     */
    explicit SlidingModeController(rclcpp::Node* node);

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
    std::string getName() const override { return "SlidingMode"; }

private:
    // Sliding mode control parameters
    double lambda_{1.0};      // Sliding surface slope
    double k_{2.0};           // Reaching law gain
    double eta_{0.5};         // Switching gain
    double epsilon_{0.05};    // Boundary layer thickness

    // Velocity control parameters
    double base_velocity_{1.0};     // Base velocity [m/s]
    double curvature_gain_{2.0};    // Curvature scaling factor

    // State tracking
    double prev_cte_{0.0};
    double prev_time_{0.0};
    bool first_call_{true};

    rclcpp::Node* node_;

    /**
     * @brief Smooth sign function (tanh approximation)
     * Reduces chattering compared to sign()
     */
    double smoothSign(double x, double epsilon) const {
        return std::tanh(x / epsilon);
    }
};

} // namespace path_follower

#endif // SLIDING_MODE_CONTROLLER_HPP
