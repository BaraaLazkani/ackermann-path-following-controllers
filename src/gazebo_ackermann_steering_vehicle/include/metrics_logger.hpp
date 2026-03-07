#ifndef METRICS_LOGGER_HPP
#define METRICS_LOGGER_HPP

#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include "path_follower_base.hpp"

namespace path_follower {

/**
 * @brief Logs performance metrics to CSV file
 *
 * Records state, reference, errors, and control commands at each time step.
 * Computes summary statistics (mean, RMS, max) for performance analysis.
 */
class MetricsLogger {
public:
    /**
     * @brief Constructor
     * @param filename Path to CSV output file
     */
    explicit MetricsLogger(const std::string& filename);

    /**
     * @brief Destructor - closes file and writes summary
     */
    ~MetricsLogger();

    /**
     * @brief Log data for one time step
     * @param timestamp Current time [s]
     * @param state Current vehicle state
     * @param reference Reference state
     * @param cte Cross-track error [m]
     * @param cmd Control commands
     */
    void log(double timestamp,
             const VehicleState& state,
             const ReferenceState& reference,
             double cte,
             const ControlCommand& cmd);

    /**
     * @brief Close the log file and compute statistics
     */
    void close();

    /**
     * @brief Get summary statistics
     */
    struct Statistics {
        double mean_cte;
        double rms_cte;
        double max_cte;
        double mean_abs_steering;
        double max_abs_steering;
        size_t num_samples;
    };

    Statistics getStatistics() const { return stats_; }

    /**
     * @brief Print statistics to console
     */
    void printStatistics() const;

private:
    std::ofstream file_;
    std::string filename_;
    bool is_open_;

    // Running statistics
    std::vector<double> cte_values_;
    std::vector<double> steering_values_;
    Statistics stats_;

    /**
     * @brief Compute statistics from logged data
     */
    void computeStatistics();
};

} // namespace path_follower

#endif // METRICS_LOGGER_HPP
