# Ackermann Steering Vehicle Path-Following Controllers

A ROS 2 simulation package that implements and compares three path-following control algorithms — **PID**, **Stanley**, and **Sliding Mode Control (SMC)** — on an Ackermann steering vehicle in Gazebo Harmonic. The project was developed as part of a Multivariable Control Systems course at Manara University.

> **Authors:** Ammar Daher, Baraa Lazkani, Humam Yehia
> **Institution:** Manara University
> **Contact:** lazkani.baraa.official@gmail.com

---

## Table of Contents

- [Overview](#overview)
- [Vehicle Model](#vehicle-model)
- [Simulation Environment](#simulation-environment)
- [Controllers](#controllers)
  - [PID Controller](#1-pid-controller)
  - [Stanley Controller](#2-stanley-controller)
  - [Sliding Mode Controller (SMC)](#3-sliding-mode-controller-smc)
  - [LQR Controller (Implemented, Results Pending)](#4-lqr-controller-implemented-results-pending)
- [Test Scenarios](#test-scenarios)
- [Performance Metrics](#performance-metrics)
- [Results & Comparison](#results--comparison)
  - [Lane Change](#lane-change)
  - [Circle](#circle)
  - [Summary Table](#summary-table)
- [Individual Controller Plots](#individual-controller-plots)
  - [PID](#pid-controller-plots)
  - [Stanley](#stanley-controller-plots)
  - [SMC](#smc-controller-plots)
- [Project Structure](#project-structure)
- [Dependencies](#dependencies)
- [Build & Run](#build--run)
- [Report](#report)
- [License](#license)

---

## Overview

This project implements a modular path-following framework for an Ackermann steering vehicle simulated in Gazebo Harmonic with ROS 2 Jazzy. The framework allows any controller to be swapped in at runtime via a launch argument. Each controller receives the vehicle's current state (pose, velocity) and a reference waypoint from the path, then outputs a steering angle and velocity command.

The comparison evaluates three control strategies across two test trajectories (lane change and circle), measuring tracking accuracy and steering smoothness.

---

## Vehicle Model

The simulated vehicle uses a standard Ackermann steering geometry modeled in Xacro/URDF:

| Parameter | Value |
|-----------|-------|
| Wheelbase | 0.96 m |
| Track width | 0.58 m |
| Max steering angle | ±0.6 rad (~34°) |
| Max velocity | 2.0 m/s |
| Front camera | 640×480, 80° FOV |

The vehicle is actuated via `gz_ros2_control` with position-controlled steering joints and velocity-controlled wheel joints. A front-facing camera is included for potential perception tasks.

---

## Simulation Environment

| Component | Version |
|-----------|---------|
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| C++ Standard | C++17 |
| Eigen3 | 3.x |
| Control Rate | 50 Hz |

State estimation uses TF2 to extract pose from the `odom → base_link` transform published by Gazebo's odometry plugin. All controllers run at 50 Hz.

---

## Controllers

All controllers inherit from the abstract base class `PathFollowerBase` and implement:
- `computeControl(VehicleState, ReferenceState) → ControlCommand`
- `reset()` — clears integrators/derivative state
- `getName()` — returns controller identifier

The **VehicleState** contains: `(x, y, θ, v)` — position, heading, and speed.
The **ReferenceState** contains: `(x_ref, y_ref, θ_ref, κ_ref, v_ref)` — reference position, heading, curvature, and velocity.

### 1. PID Controller

The PID controller maps cross-track error (CTE) directly to a steering command using three terms:

```
e(t) = cross-track error (signed perpendicular distance to path)

δ(t) = Kp·e(t) + Ki·∫e dt + Kd·de/dt
```

The velocity is reduced on curves via a curvature gain:
```
v_cmd = base_velocity - curvature_gain · |κ_ref|
```

**Parameters** (`config/pid_params.yaml`):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `kp` | 4.0 | Proportional gain |
| `ki` | 0.2 | Integral gain (with anti-windup) |
| `kd` | 1.0 | Derivative gain |
| `base_velocity` | 1.0 m/s | Nominal speed |
| `curvature_gain` | 0.5 | Speed reduction on curves |

---

### 2. Stanley Controller

The Stanley controller, developed at Stanford for the DARPA Grand Challenge, combines heading error and a velocity-adaptive cross-track correction:

```
δ = θ_e + arctan(k · e / (k_s + v))
```

Where:
- `θ_e` — heading error (difference between vehicle heading and path tangent)
- `e` — cross-track error at the front axle
- `v` — vehicle speed
- `k` — cross-track gain
- `k_s` — softening constant (prevents singularity at low speed)

The `arctan` term naturally diminishes at high speed (steering less aggressively when fast) and increases at low speed (correcting lateral deviation more assertively).

**Parameters** (`config/stanley_params.yaml`):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `k` | 2.5 | Cross-track gain |
| `k_s` | 1.0 | Softening constant |
| `base_velocity` | 1.0 m/s | Nominal speed |

---

### 3. Sliding Mode Controller (SMC)

The Sliding Mode Controller is a robust nonlinear method. It defines a sliding surface `s` that combines heading error and cross-track error, then drives the system to this surface and maintains it.

```
Sliding surface:
  s = θ_e + λ · e

Control law:
  δ = -k · s - η · sat(s/ε)
```

Where:
- `λ` — slope of the sliding surface (trade-off between heading and CTE correction)
- `k` — reaching gain (how fast to reach the surface)
- `η` — switching gain
- `ε` — boundary layer thickness (replaces hard `sign(s)` with `sat(s/ε)` to reduce chattering)

**Parameters** (`config/smc_params.yaml`):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `lambda` | 1.5 | Sliding surface slope |
| `k` | 2.0 | Reaching gain |
| `eta` | 0.5 | Switching gain |
| `epsilon` | 0.1 | Boundary layer (chattering reduction) |

---

### 4. LQR Controller (Implemented, Results Pending)

A Linear Quadratic Regulator is also implemented (`lqr_controller.hpp/cpp`). It linearizes the bicycle kinematic model and solves the continuous-time Algebraic Riccati Equation (CARE) online to compute optimal state-feedback gains.

State vector: `[e, ė, θ_e, θ̇_e]ᵀ` (cross-track error, its rate, heading error, its rate)

Experimental results for LQR will be added in a future update.

---

## Test Scenarios

### Lane Change

A smooth lateral displacement trajectory (S-curve) simulating a highway lane change maneuver. The path requires the vehicle to shift sideways by ~2 m over ~20 m of forward travel.

- **Speed:** 1.0 m/s
- **Challenge:** Smooth heading transitions, minimal overshoot

### Circle

A circular path with a fixed radius, requiring a constant non-zero steering angle to maintain.

- **Speed:** ~0.3 m/s (reduced for tight radius)
- **Radius:** ~1.5 m
- **Challenge:** Sustained curvature tracking, steady-state CTE

---

## Performance Metrics

| Metric | Symbol | Definition |
|--------|--------|-----------|
| **Mean CTE** | $\bar{e}$ | Average absolute cross-track error over the run [m] — overall accuracy |
| **RMS CTE** | $e_\text{rms}$ | Root-mean-square of CTE [m] — penalizes large deviations more heavily than mean |
| **Max CTE** | $e_\text{max}$ | Worst-case absolute cross-track error [m] — robustness indicator |
| **Mean δ** | $\bar{\delta}$ | Average absolute steering angle [°] — indicator of control effort and smoothness |

---

## Results & Comparison

### Lane Change

#### Comparison Plot

![Lane Change Comparison](results/plots/comparison/comparison_lane_change.png)

#### Metrics

| Controller | Mean CTE [m] | RMS CTE [m] | Max CTE [m] | Mean δ [°] |
|------------|:------------:|:-----------:|:-----------:|:----------:|
| **Stanley** | **0.0052** | **0.0082** | **0.0238** | **3.07** |
| PID | 0.0186 | 0.0314 | 0.0923 | 5.86 |
| SMC | 0.0387 | 0.0615 | 0.1770 | 13.35 |

**Key observations (Lane Change):**
- Stanley achieves the best accuracy on lane change — **3.6× better mean CTE than PID** and **7.4× better than SMC**
- Stanley also requires the least steering effort (3.07°), indicating smooth, well-coordinated control
- SMC exhibits significantly higher steering activity (13.35°) due to the switching control nature, even with boundary layer smoothing
- PID performs reasonably with moderate gains but lacks the heading-compensation mechanism of Stanley

---

### Circle

#### Comparison Plot

![Circle Comparison](results/plots/comparison/comparison_circle.png)

#### Metrics

| Controller | Mean CTE [m] | RMS CTE [m] | Max CTE [m] | Mean δ [°] |
|------------|:------------:|:-----------:|:-----------:|:----------:|
| **PID** | **0.0308** | **0.0531** | **0.2329** | **11.50** |
| Stanley | 0.0488 | 0.0823 | 0.3308 | 19.25 |
| SMC | 0.1297 | 0.1545 | 0.5192 | 23.87 |

**Key observations (Circle):**
- On the circular trajectory, PID performs best with the lowest mean CTE (0.031 m)
- Stanley struggles more on the circle than on the lane change — the curvature-adaptive term of the arctan may cause oversteering at the constant tight curvature
- SMC shows the highest errors on the circle (mean CTE 0.130 m), suggesting the sliding surface parameters are not optimal for sustained high-curvature paths
- All controllers show higher CTE on the circle than on lane change, reflecting the difficulty of tight constant-curvature tracking

---

### Summary Table

| Controller | Lane Change Mean CTE | Circle Mean CTE | Overall Rank |
|------------|:-------------------:|:---------------:|:------------:|
| **Stanley** | **0.0052 m** | 0.0488 m | 1 |
| **PID** | 0.0186 m | **0.0308 m** | 2 |
| SMC | 0.0387 m | 0.1297 m | 3 |

> **Conclusion:** Stanley is the best overall performer for smooth trajectories (lane change). PID is the most robust across different trajectory types. SMC, while theoretically robust to disturbances, requires more careful parameter tuning to match the performance of the other two on these test trajectories.

---

## Individual Controller Plots

### PID Controller Plots

#### Lane Change

| Trajectory | CTE vs Time | Heading Error |
|:----------:|:-----------:|:-------------:|
| ![](results/plots/pid/pid_lane_change_01_trajectory.png) | ![](results/plots/pid/pid_lane_change_02_cte_time.png) | ![](results/plots/pid/pid_lane_change_03_heading_error.png) |

| Steering Command | Velocity Tracking | Position Components |
|:----------------:|:-----------------:|:-------------------:|
| ![](results/plots/pid/pid_lane_change_04_steering_command.png) | ![](results/plots/pid/pid_lane_change_05_velocity_tracking.png) | ![](results/plots/pid/pid_lane_change_06_position_components.png) |

| Curvature & Velocity | Error Histograms | Steering Rate |
|:--------------------:|:----------------:|:-------------:|
| ![](results/plots/pid/pid_lane_change_07_curvature_velocity.png) | ![](results/plots/pid/pid_lane_change_08_error_histograms.png) | ![](results/plots/pid/pid_lane_change_10_steering_rate.png) |

#### Circle

| Trajectory | CTE vs Time | Heading Error |
|:----------:|:-----------:|:-------------:|
| ![](results/plots/pid/pid_circle_01_trajectory.png) | ![](results/plots/pid/pid_circle_02_cte_time.png) | ![](results/plots/pid/pid_circle_03_heading_error.png) |

| Steering Command | Position Components | Error Histograms |
|:----------------:|:-------------------:|:----------------:|
| ![](results/plots/pid/pid_circle_04_steering_command.png) | ![](results/plots/pid/pid_circle_06_position_components.png) | ![](results/plots/pid/pid_circle_08_error_histograms.png) |

---

### Stanley Controller Plots

#### Lane Change

| Trajectory | CTE vs Time | Heading Error |
|:----------:|:-----------:|:-------------:|
| ![](results/plots/stanley/stanley_lane_change_01_trajectory.png) | ![](results/plots/stanley/stanley_lane_change_02_cte_time.png) | ![](results/plots/stanley/stanley_lane_change_03_heading_error.png) |

| Steering Command | Velocity Tracking | Position Components |
|:----------------:|:-----------------:|:-------------------:|
| ![](results/plots/stanley/stanley_lane_change_04_steering_command.png) | ![](results/plots/stanley/stanley_lane_change_05_velocity_tracking.png) | ![](results/plots/stanley/stanley_lane_change_06_position_components.png) |

| Curvature & Velocity | Error Histograms | Steering Rate |
|:--------------------:|:----------------:|:-------------:|
| ![](results/plots/stanley/stanley_lane_change_07_curvature_velocity.png) | ![](results/plots/stanley/stanley_lane_change_08_error_histograms.png) | ![](results/plots/stanley/stanley_lane_change_10_steering_rate.png) |

#### Circle

| Trajectory | CTE vs Time | Heading Error |
|:----------:|:-----------:|:-------------:|
| ![](results/plots/stanley/stanley_circle_01_trajectory.png) | ![](results/plots/stanley/stanley_circle_02_cte_time.png) | ![](results/plots/stanley/stanley_circle_03_heading_error.png) |

| Steering Command | Position Components | Error Histograms | Steering Rate |
|:----------------:|:-------------------:|:----------------:|:-------------:|
| ![](results/plots/stanley/stanley_circle_04_steering_command.png) | ![](results/plots/stanley/stanley_circle_06_position_components.png) | ![](results/plots/stanley/stanley_circle_08_error_histograms.png) | ![](results/plots/stanley/stanley_circle_10_steering_rate.png) |

---

### SMC Controller Plots

#### Lane Change

| Trajectory | CTE vs Time | Heading Error |
|:----------:|:-----------:|:-------------:|
| ![](results/plots/smc/smc_lane_change_01_trajectory.png) | ![](results/plots/smc/smc_lane_change_02_cte_time.png) | ![](results/plots/smc/smc_lane_change_03_heading_error.png) |

| Steering Command | Velocity Tracking | Position Components |
|:----------------:|:-----------------:|:-------------------:|
| ![](results/plots/smc/smc_lane_change_04_steering_command.png) | ![](results/plots/smc/smc_lane_change_05_velocity_tracking.png) | ![](results/plots/smc/smc_lane_change_06_position_components.png) |

| Curvature & Velocity | Error Histograms | Steering Rate |
|:--------------------:|:----------------:|:-------------:|
| ![](results/plots/smc/smc_lane_change_07_curvature_velocity.png) | ![](results/plots/smc/smc_lane_change_08_error_histograms.png) | ![](results/plots/smc/smc_lane_change_10_steering_rate.png) |

#### Circle

| Trajectory | CTE vs Time | Heading Error |
|:----------:|:-----------:|:-------------:|
| ![](results/plots/smc/smc_circle_01_trajectory.png) | ![](results/plots/smc/smc_circle_02_cte_time.png) | ![](results/plots/smc/smc_circle_03_heading_error.png) |

| Steering Command | Position Components | Error Histograms | Steering Rate |
|:----------------:|:-------------------:|:----------------:|:-------------:|
| ![](results/plots/smc/smc_circle_04_steering_command.png) | ![](results/plots/smc/smc_circle_06_position_components.png) | ![](results/plots/smc/smc_circle_08_error_histograms.png) | ![](results/plots/smc/smc_circle_10_steering_rate.png) |

---

## Project Structure

```
ackermann-path-following-controllers/
├── README.md
├── LICENSE
├── report/
│   ├── final_report.pdf          # Full project report
│   └── presentation.tex          # LaTeX Beamer presentation source
├── results/
│   ├── data/                     # Raw CSV logs
│   │   ├── pid_lane_change.csv
│   │   ├── pid_circle.csv
│   │   ├── stanley_lane_change.csv
│   │   ├── stanley_circle.csv
│   │   ├── smc_lane_change.csv
│   │   └── smc_circle.csv
│   └── plots/
│       ├── comparison/           # Side-by-side controller comparisons
│       ├── pid/                  # Individual PID plots
│       ├── stanley/              # Individual Stanley plots
│       └── smc/                  # Individual SMC plots
└── src/
    └── gazebo_ackermann_steering_vehicle/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/              # C++ headers
        │   ├── path_follower_base.hpp
        │   ├── pid_controller.hpp
        │   ├── stanley_controller.hpp
        │   ├── sliding_mode_controller.hpp
        │   ├── lqr_controller.hpp
        │   ├── path_manager.hpp
        │   ├── state_estimator.hpp
        │   ├── trajectory_generator.hpp
        │   ├── metrics_logger.hpp
        │   └── vehicle_controller.hpp
        ├── src/                  # C++ implementations
        ├── config/               # YAML parameter files
        ├── launch/               # ROS 2 launch files
        ├── msg/                  # Custom message definitions
        ├── model/                # Vehicle URDF/Xacro
        └── scripts/              # Python analysis tools
```

---

## Dependencies

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop

# Gazebo Harmonic + ROS bridge
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control

# TF2
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs

# Eigen3 (for LQR)
sudo apt install libeigen3-dev

# Joystick (optional)
sudo apt install ros-jazzy-joy
```

---

## Build & Run

### Build

```bash
# From the repo root (workspace root is src/)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select gazebo_ackermann_steering_vehicle
source install/setup.bash
```

### Launch Vehicle

```bash
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py
```

### Run a Controller

```bash
# controller: pid | stanley | smc | lqr
# trajectory: lane_change | circle | figure8 | s_curve

ros2 launch gazebo_ackermann_steering_vehicle path_follower.launch.py \
  controller:=stanley trajectory:=lane_change
```

### Joystick Control

```bash
ros2 launch gazebo_ackermann_steering_vehicle joystick.launch.py
```

### Visualize Results

```bash
# Single controller run
python3 src/gazebo_ackermann_steering_vehicle/scripts/plot_results.py \
  results/data/stanley_lane_change.csv

# Compare all controllers
python3 src/gazebo_ackermann_steering_vehicle/scripts/compare_controllers.py \
  results/data lane_change
```

---

## Report

The full project report and presentation are available in the [`report/`](report/) directory:

- [`report/final_report.pdf`](report/final_report.pdf) — Complete analysis, methodology, and results
- [`report/presentation.tex`](report/presentation.tex) — LaTeX Beamer source for the project presentation

---

## Acknowledgements

The project, implementation, experiments, and analysis were done entirely by the authors.
Claude AI (Anthropic) assisted with organizing the repository structure and writing this README.

---

## License

This project is licensed under the **Ackermann Path-Following Controllers License**.
See the [LICENSE](LICENSE) file for full terms.

**Summary:** Free for personal, academic, and non-commercial use. Commercial use requires prior written permission from the authors.
Contact: **lazkani.baraa.official@gmail.com**
