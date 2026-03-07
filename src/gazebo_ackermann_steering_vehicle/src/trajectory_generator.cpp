#include "trajectory_generator.hpp"
#include <cmath>

namespace path_follower {

std::vector<Waypoint> TrajectoryGenerator::generateFigure8(
    double amplitude, size_t num_points, double velocity) {

    std::vector<Waypoint> waypoints;
    waypoints.reserve(num_points);

    // Figure-8 parametric equations: x = A*sin(t), y = A*sin(2*t)
    // Period is 2*pi
    for (size_t i = 0; i < num_points; ++i) {
        double t = 2.0 * M_PI * i / num_points;

        Waypoint wp;
        wp.x = amplitude * std::sin(t);
        wp.y = amplitude * std::sin(2.0 * t);

        // Compute heading from derivatives
        double dx_dt = amplitude * std::cos(t);
        double dy_dt = 2.0 * amplitude * std::cos(2.0 * t);
        wp.theta = std::atan2(dy_dt, dx_dt);

        wp.v_ref = velocity;
        wp.kappa = 0.0;  // Will be computed later
        wp.s = 0.0;       // Will be computed later

        waypoints.push_back(wp);
    }

    computeCurvature(waypoints);
    computeArcLength(waypoints);

    return waypoints;
}

std::vector<Waypoint> TrajectoryGenerator::generateLaneChange(
    double straight_length, double lateral_offset, double transition_length,
    size_t num_points, double velocity) {

    std::vector<Waypoint> waypoints;
    waypoints.reserve(num_points);

    double total_length = 2.0 * straight_length + transition_length;

    for (size_t i = 0; i < num_points; ++i) {
        double s = total_length * i / (num_points - 1);

        Waypoint wp;

        if (s < straight_length) {
            // First straight section
            wp.x = s;
            wp.y = 0.0;
            wp.theta = 0.0;
        } else if (s < straight_length + transition_length) {
            // Transition using 5th-order polynomial
            double s_local = s - straight_length;
            double tau = s_local / transition_length;  // normalized [0, 1]

            // 5th-order polynomial: y(tau) = offset * (10*tau^3 - 15*tau^4 + 6*tau^5)
            double tau2 = tau * tau;
            double tau3 = tau2 * tau;
            double tau4 = tau3 * tau;
            double tau5 = tau4 * tau;

            double y_poly = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            double dy_dtau = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);

            wp.x = s;
            wp.y = lateral_offset * y_poly;
            wp.theta = std::atan2(lateral_offset * dy_dtau / transition_length, 1.0);
        } else {
            // Second straight section
            wp.x = s;
            wp.y = lateral_offset;
            wp.theta = 0.0;
        }

        wp.v_ref = velocity;
        wp.kappa = 0.0;  // Will be computed later
        wp.s = 0.0;       // Will be computed later

        waypoints.push_back(wp);
    }

    computeCurvature(waypoints);
    computeArcLength(waypoints);

    return waypoints;
}

std::vector<Waypoint> TrajectoryGenerator::generateCircle(
    double radius, size_t num_points, double velocity) {

    std::vector<Waypoint> waypoints;
    waypoints.reserve(num_points);

    // Start circle at origin (0, 0) by offsetting by -radius in x
    for (size_t i = 0; i < num_points; ++i) {
        double theta = 2.0 * M_PI * i / num_points;

        Waypoint wp;
        wp.x = radius * std::cos(theta) - radius;  // Offset to start at origin
        wp.y = radius * std::sin(theta);
        wp.theta = theta + M_PI / 2.0;  // Tangent to circle
        wp.kappa = 1.0 / radius;         // Constant curvature
        wp.v_ref = velocity;
        wp.s = 0.0;  // Will be computed later

        waypoints.push_back(wp);
    }

    computeArcLength(waypoints);

    return waypoints;
}

std::vector<Waypoint> TrajectoryGenerator::generateSCurve(
    double length, double amplitude, size_t num_points, double velocity) {

    std::vector<Waypoint> waypoints;
    waypoints.reserve(num_points);

    // S-curve: y = amplitude * sin(2*pi*x/length)
    for (size_t i = 0; i < num_points; ++i) {
        double x = length * i / (num_points - 1);

        Waypoint wp;
        wp.x = x;
        wp.y = amplitude * std::sin(2.0 * M_PI * x / length);

        // Compute heading from derivative
        double dy_dx = amplitude * (2.0 * M_PI / length) * std::cos(2.0 * M_PI * x / length);
        wp.theta = std::atan(dy_dx);

        wp.v_ref = velocity;
        wp.kappa = 0.0;  // Will be computed later
        wp.s = 0.0;       // Will be computed later

        waypoints.push_back(wp);
    }

    computeCurvature(waypoints);
    computeArcLength(waypoints);

    return waypoints;
}

void TrajectoryGenerator::computeCurvature(std::vector<Waypoint>& waypoints) {
    if (waypoints.size() < 3) {
        return;
    }

    // Compute curvature using finite differences
    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        double theta_prev = waypoints[i-1].theta;
        double theta_next = waypoints[i+1].theta;

        double dx = waypoints[i+1].x - waypoints[i-1].x;
        double dy = waypoints[i+1].y - waypoints[i-1].y;
        double ds = std::sqrt(dx * dx + dy * dy);

        if (ds > 1e-6) {
            double dtheta = normalizeAngle(theta_next - theta_prev);
            waypoints[i].kappa = dtheta / ds;
        } else {
            waypoints[i].kappa = 0.0;
        }
    }

    // Set endpoints
    waypoints[0].kappa = waypoints[1].kappa;
    waypoints.back().kappa = waypoints[waypoints.size() - 2].kappa;
}

void TrajectoryGenerator::computeArcLength(std::vector<Waypoint>& waypoints) {
    if (waypoints.empty()) {
        return;
    }

    waypoints[0].s = 0.0;

    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i].x - waypoints[i-1].x;
        double dy = waypoints[i].y - waypoints[i-1].y;
        double ds = std::sqrt(dx * dx + dy * dy);
        waypoints[i].s = waypoints[i-1].s + ds;
    }
}

} // namespace path_follower
