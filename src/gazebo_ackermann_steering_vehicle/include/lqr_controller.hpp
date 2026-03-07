#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "path_follower_base.hpp"

namespace path_follower {

/**
 * @brief LQR controller for path following
 *
 * State: x = [e, ė, θ_e, θ̇_e]ᵀ
 * Linearized bicycle model for optimal control
 */
class LQRController : public PathFollowerBase {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for parameter retrieval
     */
    explicit LQRController(rclcpp::Node* node);

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
    std::string getName() const override { return "LQR"; }

private:
    // LQR cost matrices
    Eigen::Matrix4d Q_;  // State cost
    double R_;           // Control cost

    // Velocity control parameters
    double base_velocity_{1.0};     // Base velocity [m/s]
    double curvature_gain_{2.0};    // Curvature scaling factor

    // State tracking
    double prev_cte_{0.0};
    double prev_heading_error_{0.0};
    double prev_time_{0.0};
    bool first_call_{true};

    rclcpp::Node* node_;

    /**
     * @brief Solve continuous-time Algebraic Riccati Equation (CARE)
     * @param A State matrix
     * @param B Input matrix
     * @param Q State cost matrix
     * @param R Control cost scalar
     * @return Solution matrix P
     */
    Eigen::Matrix4d solveCARE(const Eigen::Matrix4d& A,
                              const Eigen::Vector4d& B,
                              const Eigen::Matrix4d& Q,
                              double R);

    /**
     * @brief Compute LQR gain matrix K
     * @param velocity Current velocity [m/s]
     * @return Gain vector K (1x4)
     */
    Eigen::RowVector4d computeLQRGain(double velocity);
};

} // namespace path_follower

#endif // LQR_CONTROLLER_HPP
