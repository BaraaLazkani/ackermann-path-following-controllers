#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <string>
#include <vector>

#include "path_follower_base.hpp"
#include "path_manager.hpp"
#include "state_estimator.hpp"
#include "metrics_logger.hpp"
#include "trajectory_generator.hpp"

// Controller includes
#include "pid_controller.hpp"
#include "stanley_controller.hpp"
#include "lqr_controller.hpp"
#include "sliding_mode_controller.hpp"

using namespace path_follower;

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("path_follower_node") {
        // Declare parameters
        this->declare_parameter<std::string>("controller_type", "pid");
        this->declare_parameter<std::string>("trajectory_type", "figure8");
        this->declare_parameter<std::string>("output_file", "test_data/results/path_follower.csv");
        this->declare_parameter<double>("control_frequency", 50.0);
        this->declare_parameter<double>("wheelbase", 0.22);
        this->declare_parameter<double>("max_steering_angle", 0.6109);
        this->declare_parameter<double>("max_velocity", 2.0);

        // Get parameters
        std::string controller_type = this->get_parameter("controller_type").as_string();
        std::string trajectory_type = this->get_parameter("trajectory_type").as_string();
        double control_freq = this->get_parameter("control_frequency").as_double();
        double wheelbase = this->get_parameter("wheelbase").as_double();
        double max_steering = this->get_parameter("max_steering_angle").as_double();
        double max_vel = this->get_parameter("max_velocity").as_double();

        // Construct output filename dynamically
        std::string output_file = "test_data/results/" + controller_type + "_" +
                                  trajectory_type + ".csv";

        RCLCPP_INFO(this->get_logger(), "Path Follower Node Starting...");
        RCLCPP_INFO(this->get_logger(), "Controller: %s", controller_type.c_str());
        RCLCPP_INFO(this->get_logger(), "Trajectory: %s", trajectory_type.c_str());
        RCLCPP_INFO(this->get_logger(), "Output file: %s", output_file.c_str());

        // Initialize components
        path_manager_ = std::make_shared<PathManager>();
        state_estimator_ = std::make_shared<StateEstimator>(this);
        metrics_logger_ = std::make_shared<MetricsLogger>(output_file);

        // Generate trajectory
        generateTrajectory(trajectory_type);

        // Create controller
        createController(controller_type, wheelbase, max_steering, max_vel);

        // Publishers
        steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/velocity", 10);
        ref_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_visualization/reference", 10);
        actual_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_visualization/actual", 10);

        // Publish reference path visualization
        publishReferencePath();

        // Control timer
        auto period = std::chrono::duration<double>(1.0 / control_freq);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&PathFollowerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Path Follower Node initialized successfully");
    }

    ~PathFollowerNode() {
        // Close metrics logger
        if (metrics_logger_) {
            metrics_logger_->close();
        }
    }

private:
    void generateTrajectory(const std::string& type) {
        std::vector<Waypoint> waypoints;

        if (type == "figure8") {
            waypoints = TrajectoryGenerator::generateFigure8(1.0, 200, 1.0);
            RCLCPP_INFO(this->get_logger(), "Generated Figure-8 trajectory with %zu waypoints (SMALL: 1m amplitude)",
                       waypoints.size());
        } else if (type == "lane_change") {
            waypoints = TrajectoryGenerator::generateLaneChange(4.0, 2.0, 2.0, 150, 1.5);
            RCLCPP_INFO(this->get_logger(), "Generated Lane Change trajectory with %zu waypoints (4m + 2m lateral + 4m)",
                       waypoints.size());
        } else if (type == "circle") {
            waypoints = TrajectoryGenerator::generateCircle(1.5, 100, 1.0);
            RCLCPP_INFO(this->get_logger(), "Generated Circle trajectory with %zu waypoints (SMALL: 1.5m radius)",
                       waypoints.size());
        } else if (type == "s_curve") {
            waypoints = TrajectoryGenerator::generateSCurve(10.0, 2.0, 150, 1.0);
            RCLCPP_INFO(this->get_logger(), "Generated S-Curve trajectory with %zu waypoints",
                       waypoints.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown trajectory type: %s", type.c_str());
            waypoints = TrajectoryGenerator::generateFigure8(2.0, 200, 1.0);
        }

        path_manager_->setPath(waypoints);

        // Log first few waypoints for verification
        RCLCPP_INFO(this->get_logger(), "First 3 waypoints:");
        for (size_t i = 0; i < std::min(size_t(3), waypoints.size()); ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] x=%.2f, y=%.2f, theta=%.2f",
                       i, waypoints[i].x, waypoints[i].y, waypoints[i].theta);
        }
    }

    void createController(const std::string& type, double wheelbase,
                         double max_steering, double max_vel) {
        if (type == "pid") {
            controller_ = std::make_shared<PIDController>(this);
        } else if (type == "stanley") {
            controller_ = std::make_shared<StanleyController>(this);
        } else if (type == "lqr") {
            controller_ = std::make_shared<LQRController>(this);
        } else if (type == "smc") {
            controller_ = std::make_shared<SlidingModeController>(this);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown controller type: %s", type.c_str());
            controller_ = std::make_shared<PIDController>(this);
        }

        controller_->initialize(wheelbase, max_steering, max_vel);
        RCLCPP_INFO(this->get_logger(), "Controller '%s' initialized", controller_->getName().c_str());
    }

    void controlLoop() {
        // Get current state
        VehicleState state;
        if (!state_estimator_->getState(state)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Failed to get vehicle state from TF!");
            return;  // TF not available yet
        }

        // Log successful state acquisition once
        static bool state_logged = false;
        if (!state_logged) {
            RCLCPP_INFO(this->get_logger(),
                       "Successfully getting state: x=%.2f, y=%.2f, theta=%.2f",
                       state.x, state.y, state.theta);
            state_logged = true;
        }

        // Find closest point on path
        size_t closest_idx = path_manager_->findClosestPoint(state.x, state.y);

        // Get reference state
        ReferenceState reference = path_manager_->getReferenceState(closest_idx);

        // Compute cross-track error
        double cte = path_manager_->computeCrossTrackError(state.x, state.y, closest_idx);

        // Compute control
        ControlCommand cmd = controller_->computeControl(state, reference, cte);

        // Log control commands periodically
        static int log_counter = 0;
        if (log_counter++ % 50 == 0) {  // Log every 50 iterations (~1 second at 50Hz)
            RCLCPP_INFO(this->get_logger(),
                       "CTE: %.3f, Steering: %.3f, Velocity: %.3f",
                       cte, cmd.steering_angle, cmd.velocity);
        }

        // Publish commands
        std_msgs::msg::Float64 steering_msg;
        steering_msg.data = cmd.steering_angle;
        steering_pub_->publish(steering_msg);

        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = cmd.velocity;
        velocity_pub_->publish(velocity_msg);

        // Log metrics
        metrics_logger_->log(state.timestamp, state, reference, cte, cmd);

        // Publish actual path visualization
        publishActualPath(state.x, state.y);

        // Check if path is complete
        if (path_manager_->isPathComplete(closest_idx, 0.5)) {
            static bool completion_logged = false;
            if (!completion_logged) {
                RCLCPP_INFO(this->get_logger(), "Path following complete!");
                completion_logged = true;
            }
        }
    }

    // Components
    std::shared_ptr<PathManager> path_manager_;
    std::shared_ptr<StateEstimator> state_estimator_;
    std::shared_ptr<MetricsLogger> metrics_logger_;
    std::shared_ptr<PathFollowerBase> controller_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr actual_path_pub_;

    // Path visualization
    std::vector<geometry_msgs::msg::Point> actual_path_points_;
    size_t path_point_counter_{0};

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    void publishReferencePath() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "reference_path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Reference path color (green)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        // Line width
        marker.scale.x = 0.05;

        // Get waypoints from path manager
        const auto& waypoints = path_manager_->getWaypoints();

        for (const auto& wp : waypoints) {
            geometry_msgs::msg::Point p;
            p.x = wp.x;
            p.y = wp.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        ref_path_pub_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published reference path with %zu waypoints", waypoints.size());
    }

    void publishActualPath(double x, double y) {
        // Add current position to path
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.05;  // Slightly above reference path to avoid z-fighting
        actual_path_points_.push_back(p);

        // Update every 10 points to reduce visualization load
        path_point_counter_++;
        if (path_point_counter_ % 10 != 0) {
            return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "actual_path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Actual path color (red)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        // Line width
        marker.scale.x = 0.05;

        marker.points = actual_path_points_;

        actual_path_pub_->publish(marker);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
