#include "path_manager.hpp"
#include <limits>
#include <algorithm>

namespace path_follower {

void PathManager::setPath(const std::vector<Waypoint>& waypoints) {
    waypoints_ = waypoints;
    last_waypoint_idx_ = 0;  // Reset to start for new path

    // Compute arc length if not already set
    if (!waypoints_.empty() && waypoints_[0].s == 0.0) {
        waypoints_[0].s = 0.0;
        for (size_t i = 1; i < waypoints_.size(); ++i) {
            double dx = waypoints_[i].x - waypoints_[i-1].x;
            double dy = waypoints_[i].y - waypoints_[i-1].y;
            double ds = std::sqrt(dx * dx + dy * dy);
            waypoints_[i].s = waypoints_[i-1].s + ds;
        }
    }
}

size_t PathManager::findClosestPoint(double x, double y) {
    if (waypoints_.empty()) {
        return 0;
    }

    // Search forward from last waypoint to prevent backward jumps
    // Search window: from last_idx to min(last_idx + 50, end)
    size_t search_start = last_waypoint_idx_;
    size_t search_end = std::min(last_waypoint_idx_ + 50, waypoints_.size());

    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = last_waypoint_idx_;

    for (size_t i = search_start; i < search_end; ++i) {
        double dist = distance(x, y, waypoints_[i].x, waypoints_[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // Update last waypoint (only moves forward, never backward)
    last_waypoint_idx_ = closest_idx;

    return closest_idx;
}

double PathManager::computeCrossTrackError(double x, double y, size_t closest_idx) const {
    if (waypoints_.empty() || closest_idx >= waypoints_.size()) {
        return 0.0;
    }

    const Waypoint& ref = waypoints_[closest_idx];

    // Vector from reference point to current position
    double dx = x - ref.x;
    double dy = y - ref.y;

    // Path direction vector (tangent to path)
    double path_dx = std::cos(ref.theta);
    double path_dy = std::sin(ref.theta);

    // Cross-track error is the perpendicular distance
    // Positive if vehicle is to the left of the path
    double cte = -path_dx * dy + path_dy * dx;

    return cte;
}

size_t PathManager::getLookaheadPoint(size_t current_idx, double lookahead_distance) const {
    if (waypoints_.empty()) {
        return 0;
    }

    double target_s = waypoints_[current_idx].s + lookahead_distance;

    // Find waypoint closest to target arc length
    size_t lookahead_idx = current_idx;
    double min_diff = std::abs(waypoints_[current_idx].s - target_s);

    for (size_t i = current_idx; i < waypoints_.size(); ++i) {
        double diff = std::abs(waypoints_[i].s - target_s);
        if (diff < min_diff) {
            min_diff = diff;
            lookahead_idx = i;
        } else {
            // We've passed the optimal point
            break;
        }
    }

    return lookahead_idx;
}

ReferenceState PathManager::getReferenceState(size_t idx) const {
    ReferenceState ref;

    if (waypoints_.empty() || idx >= waypoints_.size()) {
        ref.x = 0.0;
        ref.y = 0.0;
        ref.theta = 0.0;
        ref.kappa = 0.0;
        ref.v_ref = 0.0;
        ref.s = 0.0;
        return ref;
    }

    const Waypoint& wp = waypoints_[idx];
    ref.x = wp.x;
    ref.y = wp.y;
    ref.theta = wp.theta;
    ref.kappa = wp.kappa;
    ref.v_ref = wp.v_ref;
    ref.s = wp.s;

    return ref;
}

bool PathManager::isPathComplete(size_t current_idx, double threshold) const {
    if (waypoints_.empty()) {
        return false;
    }

    // Check if we're within threshold of the last waypoint
    size_t last_idx = waypoints_.size() - 1;
    if (current_idx >= last_idx) {
        return true;
    }

    // Check arc length distance to end
    double distance_to_end = waypoints_[last_idx].s - waypoints_[current_idx].s;
    return distance_to_end < threshold;
}

} // namespace path_follower
