#!/usr/bin/env python3

"""
Generate synthetic test data for controller comparison
This creates realistic CSV files that simulate controller performance
"""

import numpy as np
import os
from pathlib import Path
import csv


def generate_figure8_reference(num_points=200):
    """Generate figure-8 reference trajectory"""
    t = np.linspace(0, 2*np.pi, num_points)
    amplitude = 2.0

    x_ref = amplitude * np.sin(t)
    y_ref = amplitude * np.sin(2*t)

    # Compute heading and curvature
    dx = amplitude * np.cos(t)
    dy = 2 * amplitude * np.cos(2*t)
    theta_ref = np.arctan2(dy, dx)

    # Compute curvature numerically
    kappa_ref = np.zeros_like(t)
    for i in range(1, len(t)-1):
        dtheta = theta_ref[i+1] - theta_ref[i-1]
        ds = np.sqrt((x_ref[i+1]-x_ref[i-1])**2 + (y_ref[i+1]-y_ref[i-1])**2)
        kappa_ref[i] = dtheta / ds if ds > 0 else 0

    # Arc length
    s_ref = np.zeros_like(t)
    for i in range(1, len(t)):
        ds = np.sqrt((x_ref[i]-x_ref[i-1])**2 + (y_ref[i]-y_ref[i-1])**2)
        s_ref[i] = s_ref[i-1] + ds

    return x_ref, y_ref, theta_ref, kappa_ref, s_ref


def generate_lane_change_reference(num_points=200):
    """Generate lane change reference trajectory"""
    straight_length = 5.0
    lateral_offset = 2.0
    transition_length = 4.0
    total_length = 2.0 * straight_length + transition_length

    # Generate points along total length
    s = np.linspace(0, total_length, num_points)
    x_ref = s
    y_ref = np.zeros_like(s)
    theta_ref = np.zeros_like(s)
    kappa_ref = np.zeros_like(s)

    for i, si in enumerate(s):
        if si < straight_length:
            # First straight section
            y_ref[i] = 0.0
            theta_ref[i] = 0.0
            kappa_ref[i] = 0.0
        elif si < straight_length + transition_length:
            # Transition using 5th-order polynomial
            s_local = si - straight_length
            tau = s_local / transition_length  # normalized [0, 1]

            # 5th-order polynomial
            tau2 = tau * tau
            tau3 = tau2 * tau
            tau4 = tau3 * tau
            tau5 = tau4 * tau

            y_poly = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
            dy_dtau = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4)
            d2y_dtau2 = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3)

            y_ref[i] = lateral_offset * y_poly
            dy_ds = lateral_offset * dy_dtau / transition_length
            theta_ref[i] = np.arctan(dy_ds)

            # Curvature calculation
            d2y_ds2 = lateral_offset * d2y_dtau2 / (transition_length**2)
            kappa_ref[i] = d2y_ds2 / ((1 + dy_ds**2)**1.5)
        else:
            # Second straight section
            y_ref[i] = lateral_offset
            theta_ref[i] = 0.0
            kappa_ref[i] = 0.0

    # Arc length (approximately equal to x for this trajectory)
    s_ref = s

    return x_ref, y_ref, theta_ref, kappa_ref, s_ref


def generate_controller_data(controller_name, trajectory='figure8', output_dir='test_data/results'):
    """Generate synthetic data for a specific controller"""

    # Performance characteristics for each controller
    controller_params = {
        'pid': {
            'mean_cte': 0.15,
            'cte_std': 0.08,
            'max_cte': 0.40,
            'response_delay': 0.1,
            'oscillation_freq': 0.5,
            'oscillation_amp': 0.05
        },
        'stanley': {
            'mean_cte': 0.10,
            'cte_std': 0.05,
            'max_cte': 0.30,
            'response_delay': 0.05,
            'oscillation_freq': 0.3,
            'oscillation_amp': 0.03
        },
        'lqr': {
            'mean_cte': 0.08,
            'cte_std': 0.04,
            'max_cte': 0.20,
            'response_delay': 0.02,
            'oscillation_freq': 0.2,
            'oscillation_amp': 0.02
        },
        'smc': {
            'mean_cte': 0.12,
            'cte_std': 0.06,
            'max_cte': 0.25,
            'response_delay': 0.03,
            'oscillation_freq': 0.4,
            'oscillation_amp': 0.025
        }
    }

    params = controller_params[controller_name]

    # Generate reference trajectory
    num_points = 1000
    if trajectory == 'figure8':
        x_ref, y_ref, theta_ref, kappa_ref, s_ref = generate_figure8_reference(num_points)
    elif trajectory == 'lane_change':
        x_ref, y_ref, theta_ref, kappa_ref, s_ref = generate_lane_change_reference(num_points)
    else:
        # Default to figure8
        x_ref, y_ref, theta_ref, kappa_ref, s_ref = generate_figure8_reference(num_points)

    # Time vector (50 Hz control)
    dt = 0.02
    duration = num_points * dt
    timestamp = np.arange(0, duration, dt)[:num_points]

    # Reference velocity (slow down on high curvature)
    v_ref = 1.0 * np.exp(-2.0 * np.abs(kappa_ref))

    # Simulate vehicle tracking with errors
    # Cross-track error: combination of bias, noise, and oscillation
    cte_bias = params['mean_cte'] * np.sin(2*np.pi * timestamp / duration)
    cte_noise = np.random.normal(0, params['cte_std'], num_points)
    cte_oscillation = params['oscillation_amp'] * np.sin(2*np.pi * params['oscillation_freq'] * timestamp)
    cte = cte_bias + cte_noise + cte_oscillation
    cte = np.clip(cte, -params['max_cte'], params['max_cte'])

    # Actual position (reference + error in perpendicular direction)
    x = x_ref - cte * np.sin(theta_ref)
    y = y_ref + cte * np.cos(theta_ref)

    # Heading error (smaller than CTE)
    heading_error = cte * 0.2 + np.random.normal(0, 0.05, num_points)
    theta = theta_ref - heading_error

    # Velocity (tracks reference with small delay)
    v = np.zeros_like(v_ref)
    v[0] = v_ref[0]
    for i in range(1, num_points):
        v[i] = v[i-1] + (v_ref[i] - v[i-1]) * (1 - params['response_delay'])

    # Steering command (proportional to CTE + heading error)
    if controller_name == 'pid':
        # PID has more aggressive control
        steering_cmd = -1.5 * cte - 0.5 * heading_error
    elif controller_name == 'stanley':
        # Stanley uses velocity-adaptive gain
        k = 2.5
        k_s = 1.0
        steering_cmd = heading_error + np.arctan(k * cte / (k_s + v))
    elif controller_name == 'lqr':
        # LQR has smoother control
        steering_cmd = -1.2 * cte - 0.8 * heading_error
    elif controller_name == 'smc':
        # SMC has moderate control with some chattering
        lambda_smc = 1.0
        s = heading_error + lambda_smc * cte
        steering_cmd = -2.0 * s + 0.01 * np.random.normal(0, 1, num_points)

    # Add control limits
    max_steering = 0.6109
    steering_cmd = np.clip(steering_cmd, -max_steering, max_steering)

    # Velocity command (same as reference with small adjustment)
    velocity_cmd = v_ref * (1 - 0.1 * np.abs(steering_cmd) / max_steering)

    # Save to CSV
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, f'{controller_name}_{trajectory}.csv')

    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        # Write header
        writer.writerow(['timestamp', 'x', 'y', 'theta', 'v',
                        'x_ref', 'y_ref', 'theta_ref', 'kappa_ref', 'v_ref', 's_ref',
                        'cte', 'steering_cmd', 'velocity_cmd'])
        # Write data rows
        for i in range(num_points):
            writer.writerow([timestamp[i], x[i], y[i], theta[i], v[i],
                           x_ref[i], y_ref[i], theta_ref[i], kappa_ref[i], v_ref[i], s_ref[i],
                           cte[i], steering_cmd[i], velocity_cmd[i]])

    print(f"Generated: {output_file}")
    print(f"  - Mean |CTE|: {np.mean(np.abs(cte)):.4f} m")
    print(f"  - RMS CTE: {np.sqrt(np.mean(cte**2)):.4f} m")
    print(f"  - Max |CTE|: {np.max(np.abs(cte)):.4f} m")
    print(f"  - Data points: {num_points}")

    return output_file


def main(trajectory='figure8'):
    """Generate synthetic data for all controllers"""
    print("=" * 60)
    print("Generating Synthetic Test Data")
    print(f"Trajectory: {trajectory.upper()}")
    print("=" * 60)
    print()

    controllers = ['pid', 'stanley', 'lqr', 'smc']

    base_dir = Path(__file__).parent.parent
    output_dir = base_dir / 'test_data' / 'results'

    for controller in controllers:
        print(f"\n{controller.upper()} Controller:")
        print("-" * 40)
        generate_controller_data(controller, trajectory, str(output_dir))

    print("\n" + "=" * 60)
    print("Synthetic data generation complete!")
    print("=" * 60)
    print()
    print("Next steps:")
    print(f"  1. View individual results:")
    print(f"     python3 scripts/plot_results.py {output_dir}/pid_figure8.csv")
    print()
    print(f"  2. Compare all controllers:")
    print(f"     python3 scripts/compare_controllers.py {output_dir} figure8")


if __name__ == "__main__":
    import sys
    trajectory = sys.argv[1] if len(sys.argv) > 1 else 'figure8'
    main(trajectory)
