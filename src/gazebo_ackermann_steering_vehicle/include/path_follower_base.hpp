#ifndef PATH_FOLLOWER_BASE_HPP
#define PATH_FOLLOWER_BASE_HPP

#include <string>

namespace path_follower {

/**
 * @brief Vehicle state representation
 */
struct VehicleState {
    double x;           // position x [m]
    double y;           // position y [m]
    double theta;       // heading angle [rad]
    double v;           // linear velocity [m/s]
    double timestamp;   // time in seconds
};

/**
 * @brief Reference state on the path
 */
struct ReferenceState {
    double x;           // reference position x [m]
    double y;           // reference position y [m]
    double theta;       // reference heading [rad]
    double kappa;       // path curvature [1/m]
    double v_ref;       // reference velocity [m/s]
    double s;           // arc length along path [m]
};

/**
 * @brief Control commands output
 */
struct ControlCommand {
    double steering_angle;  // steering angle [rad]
    double velocity;        // linear velocity [m/s]
};

/**
 * @brief Abstract base class for path following controllers
 *
 * All controllers must inherit from this class and implement the
 * computeControl method.
 */
class PathFollowerBase {
public:
    virtual ~PathFollowerBase() = default;

    /**
     * @brief Compute control commands given current state and reference
     * @param current_state Current vehicle state
     * @param reference_state Reference state on the path
     * @param cross_track_error Cross-track error (perpendicular distance to path) [m]
     * @return Control commands (steering angle and velocity)
     */
    virtual ControlCommand computeControl(
        const VehicleState& current_state,
        const ReferenceState& reference_state,
        double cross_track_error) = 0;

    /**
     * @brief Initialize controller with parameters
     * @param wheelbase Vehicle wheelbase [m]
     * @param max_steering_angle Maximum steering angle [rad]
     * @param max_velocity Maximum velocity [m/s]
     */
    virtual void initialize(double wheelbase, double max_steering_angle, double max_velocity) {
        wheelbase_ = wheelbase;
        max_steering_angle_ = max_steering_angle;
        max_velocity_ = max_velocity;
    }

    /**
     * @brief Reset controller internal state
     */
    virtual void reset() = 0;

    /**
     * @brief Get controller name
     */
    virtual std::string getName() const = 0;

protected:
    double wheelbase_{0.22};              // vehicle wheelbase [m]
    double max_steering_angle_{0.6109};   // max steering angle [rad]
    double max_velocity_{2.0};            // max velocity [m/s]

    /**
     * @brief Clamp value between min and max
     */
    double clamp(double value, double min_val, double max_val) const {
        return std::max(min_val, std::min(value, max_val));
    }

    /**
     * @brief Normalize angle to [-pi, pi]
     */
    double normalizeAngle(double angle) const {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

} // namespace path_follower

#endif // PATH_FOLLOWER_BASE_HPP
