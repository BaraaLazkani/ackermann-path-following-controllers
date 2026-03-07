#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <vector>
#include <cmath>
#include "path_manager.hpp"

namespace path_follower {

/**
 * @brief Generates test trajectories for path following
 *
 * Provides methods to generate common test paths:
 * - Figure-8 trajectory (variable curvature)
 * - Lane change maneuver (straight -> curve -> straight)
 * - Circular path
 */
class TrajectoryGenerator {
public:
    /**
     * @brief Generate a figure-8 trajectory
     * @param amplitude Amplitude of the figure-8 [m]
     * @param num_points Number of waypoints to generate
     * @param velocity Reference velocity [m/s]
     * @return Vector of waypoints
     */
    static std::vector<Waypoint> generateFigure8(
        double amplitude = 2.0,
        size_t num_points = 200,
        double velocity = 1.0);

    /**
     * @brief Generate a lane change trajectory
     * @param straight_length Length of straight sections [m]
     * @param lateral_offset Lateral offset for lane change [m]
     * @param transition_length Length of transition section [m]
     * @param num_points Number of waypoints to generate
     * @param velocity Reference velocity [m/s]
     * @return Vector of waypoints
     */
    static std::vector<Waypoint> generateLaneChange(
        double straight_length = 5.0,
        double lateral_offset = 2.0,
        double transition_length = 4.0,
        size_t num_points = 150,
        double velocity = 1.5);

    /**
     * @brief Generate a circular trajectory
     * @param radius Radius of circle [m]
     * @param num_points Number of waypoints to generate
     * @param velocity Reference velocity [m/s]
     * @return Vector of waypoints
     */
    static std::vector<Waypoint> generateCircle(
        double radius = 3.0,
        size_t num_points = 100,
        double velocity = 1.0);

    /**
     * @brief Generate an S-curve trajectory
     * @param length Total length of S-curve [m]
     * @param amplitude Lateral amplitude [m]
     * @param num_points Number of waypoints to generate
     * @param velocity Reference velocity [m/s]
     * @return Vector of waypoints
     */
    static std::vector<Waypoint> generateSCurve(
        double length = 10.0,
        double amplitude = 2.0,
        size_t num_points = 150,
        double velocity = 1.0);

private:
    /**
     * @brief Compute curvature numerically from waypoints
     * @param waypoints Vector of waypoints (curvature will be updated)
     */
    static void computeCurvature(std::vector<Waypoint>& waypoints);

    /**
     * @brief Compute arc length for waypoints
     * @param waypoints Vector of waypoints (arc length will be updated)
     */
    static void computeArcLength(std::vector<Waypoint>& waypoints);

    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

} // namespace path_follower

#endif // TRAJECTORY_GENERATOR_HPP
