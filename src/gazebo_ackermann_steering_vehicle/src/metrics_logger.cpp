#include "metrics_logger.hpp"
#include <iostream>
#include <iomanip>
#include <numeric>
#include <algorithm>

namespace path_follower {

MetricsLogger::MetricsLogger(const std::string& filename)
    : filename_(filename), is_open_(false) {

    file_.open(filename_);
    if (!file_.is_open()) {
        std::cerr << "Failed to open metrics log file: " << filename_ << std::endl;
        return;
    }

    is_open_ = true;

    // Write header
    file_ << "timestamp,x,y,theta,v,"
          << "x_ref,y_ref,theta_ref,kappa_ref,v_ref,s_ref,"
          << "cte,steering_cmd,velocity_cmd\n";

    file_ << std::fixed << std::setprecision(6);

    stats_ = Statistics{0.0, 0.0, 0.0, 0.0, 0.0, 0};
}

MetricsLogger::~MetricsLogger() {
    if (is_open_) {
        close();
    }
}

void MetricsLogger::log(double timestamp,
                        const VehicleState& state,
                        const ReferenceState& reference,
                        double cte,
                        const ControlCommand& cmd) {
    if (!is_open_) {
        return;
    }

    // Write data row
    file_ << timestamp << ","
          << state.x << "," << state.y << "," << state.theta << "," << state.v << ","
          << reference.x << "," << reference.y << "," << reference.theta << ","
          << reference.kappa << "," << reference.v_ref << "," << reference.s << ","
          << cte << "," << cmd.steering_angle << "," << cmd.velocity << "\n";

    // Store for statistics
    cte_values_.push_back(cte);
    steering_values_.push_back(cmd.steering_angle);
}

void MetricsLogger::close() {
    if (!is_open_) {
        return;
    }

    computeStatistics();
    file_.close();
    is_open_ = false;

    std::cout << "\nMetrics saved to: " << filename_ << std::endl;
    printStatistics();
}

void MetricsLogger::computeStatistics() {
    if (cte_values_.empty()) {
        return;
    }

    stats_.num_samples = cte_values_.size();

    // Mean absolute CTE
    double sum_abs_cte = 0.0;
    for (double cte : cte_values_) {
        sum_abs_cte += std::abs(cte);
    }
    stats_.mean_cte = sum_abs_cte / stats_.num_samples;

    // RMS CTE
    double sum_sq_cte = 0.0;
    for (double cte : cte_values_) {
        sum_sq_cte += cte * cte;
    }
    stats_.rms_cte = std::sqrt(sum_sq_cte / stats_.num_samples);

    // Max CTE
    double max_cte = 0.0;
    for (double cte : cte_values_) {
        max_cte = std::max(max_cte, std::abs(cte));
    }
    stats_.max_cte = max_cte;

    // Mean absolute steering
    double sum_abs_steering = 0.0;
    for (double steer : steering_values_) {
        sum_abs_steering += std::abs(steer);
    }
    stats_.mean_abs_steering = sum_abs_steering / stats_.num_samples;

    // Max steering
    double max_steering = 0.0;
    for (double steer : steering_values_) {
        max_steering = std::max(max_steering, std::abs(steer));
    }
    stats_.max_abs_steering = max_steering;
}

void MetricsLogger::printStatistics() const {
    std::cout << "\n=== Performance Metrics ===" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Samples:          " << stats_.num_samples << std::endl;
    std::cout << "Mean |CTE|:       " << stats_.mean_cte << " m" << std::endl;
    std::cout << "RMS CTE:          " << stats_.rms_cte << " m" << std::endl;
    std::cout << "Max |CTE|:        " << stats_.max_cte << " m" << std::endl;
    std::cout << "Mean |Steering|:  " << stats_.mean_abs_steering * 180.0 / M_PI << " deg" << std::endl;
    std::cout << "Max |Steering|:   " << stats_.max_abs_steering * 180.0 / M_PI << " deg" << std::endl;
    std::cout << "==========================\n" << std::endl;
}

} // namespace path_follower
