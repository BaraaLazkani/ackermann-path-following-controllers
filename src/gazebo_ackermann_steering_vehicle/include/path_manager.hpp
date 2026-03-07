#ifndef PATH_MANAGER_HPP
#define PATH_MANAGER_HPP

#include <vector>
#include <cmath>
#include "path_follower_base.hpp"

namespace path_follower {

/**
 * @brief Waypoint structure for path representation
 */
struct Waypoint {
    double x;           // x position [m]
    double y;           // y position [m]
    double theta;       // heading angle [rad]
    double kappa;       // curvature [1/m]
    double s;           // arc length from start [m]
    double v_ref;       // reference velocity [m/s]
};

/**
 * @brief Manages path representation and provides path following utilities
 *
 * Stores waypoints, finds closest points, computes cross-track error,
 * and provides lookahead functionality.
 */
class PathManager {
public:
    PathManager() = default;

    /**
     * @brief Set the path from a vector of waypoints
     * @param waypoints Vector of waypoints defining the path
     */
    void setPath(const std::vector<Waypoint>& waypoints);

    /**
     * @brief Find the index of the closest waypoint to the given position
     * Uses last_waypoint_idx_ to prevent backward jumps
     * @param x Current x position [m]
     * @param y Current y position [m]
     * @return Index of the closest waypoint
     */
    size_t findClosestPoint(double x, double y);

    /**
     * @brief Compute cross-track error (signed perpendicular distance to path)
     * @param x Current x position [m]
     * @param y Current y position [m]
     * @param closest_idx Index of the closest waypoint
     * @return Cross-track error [m] (positive = left of path, negative = right)
     */
    double computeCrossTrackError(double x, double y, size_t closest_idx) const;

    /**
     * @brief Get lookahead point ahead of current position on path
     * @param current_idx Current closest waypoint index
     * @param lookahead_distance Distance ahead to look [m]
     * @return Index of lookahead waypoint
     */
    size_t getLookaheadPoint(size_t current_idx, double lookahead_distance) const;

    /**
     * @brief Get reference state at a given index
     * @param idx Waypoint index
     * @return Reference state
     */
    ReferenceState getReferenceState(size_t idx) const;

    /**
     * @brief Check if path is valid (not empty)
     */
    bool isValid() const { return !waypoints_.empty(); }

    /**
     * @brief Get number of waypoints in path
     */
    size_t size() const { return waypoints_.size(); }

    /**
     * @brief Get waypoint at index
     */
    const Waypoint& getWaypoint(size_t idx) const { return waypoints_[idx]; }

    /**
     * @brief Get all waypoints
     */
    const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }

    /**
     * @brief Check if we've reached the end of the path
     * @param current_idx Current closest waypoint index
     * @param threshold Distance threshold for completion [m]
     */
    bool isPathComplete(size_t current_idx, double threshold = 0.5) const;

private:
    std::vector<Waypoint> waypoints_;
    mutable size_t last_waypoint_idx_{0};  // Track last waypoint to prevent backward jumps

    /**
     * @brief Compute distance between two points
     */
    double distance(double x1, double y1, double x2, double y2) const {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }
};

} // namespace path_follower

#endif // PATH_MANAGER_HPP
