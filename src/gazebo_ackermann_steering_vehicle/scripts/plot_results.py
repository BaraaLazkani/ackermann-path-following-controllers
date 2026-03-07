#!/usr/bin/env python3

import sys
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np


def plot_single_controller(csv_file):
    """Plot results for a single controller run"""

    # Read CSV data
    try:
        data = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: File {csv_file} not found")
        return
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))

    # 1. Trajectory plot (XY plane)
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(data['x_ref'], data['y_ref'], 'b--', linewidth=2, label='Reference Path', alpha=0.7)
    ax1.plot(data['x'], data['y'], 'r-', linewidth=1.5, label='Actual Path')
    ax1.scatter(data['x'].iloc[0], data['y'].iloc[0], c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(data['x'].iloc[-1], data['y'].iloc[-1], c='red', s=100, marker='s', label='End', zorder=5)
    ax1.set_xlabel('X Position [m]')
    ax1.set_ylabel('Y Position [m]')
    ax1.set_title('Trajectory Tracking')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # 2. Cross-track error vs time
    ax2 = plt.subplot(2, 3, 2)
    time = data['timestamp'] - data['timestamp'].iloc[0]
    ax2.plot(time, data['cte'], 'b-', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Cross-Track Error [m]')
    ax2.set_title('Cross-Track Error')
    ax2.grid(True, alpha=0.3)

    # Add statistics
    mean_cte = np.mean(np.abs(data['cte']))
    rms_cte = np.sqrt(np.mean(data['cte']**2))
    max_cte = np.max(np.abs(data['cte']))
    ax2.text(0.02, 0.98, f'Mean |CTE|: {mean_cte:.4f} m\nRMS CTE: {rms_cte:.4f} m\nMax |CTE|: {max_cte:.4f} m',
             transform=ax2.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 3. Steering angle vs time
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(time, np.degrees(data['steering_cmd']), 'g-', linewidth=1.5)
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Steering Angle [deg]')
    ax3.set_title('Steering Commands')
    ax3.grid(True, alpha=0.3)

    # Add statistics
    mean_steering = np.mean(np.abs(data['steering_cmd']))
    max_steering = np.max(np.abs(data['steering_cmd']))
    ax3.text(0.02, 0.98, f'Mean |δ|: {np.degrees(mean_steering):.2f}°\nMax |δ|: {np.degrees(max_steering):.2f}°',
             transform=ax3.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))

    # 4. Velocity vs time
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(time, data['v'], 'b-', linewidth=1.5, label='Actual')
    ax4.plot(time, data['velocity_cmd'], 'r--', linewidth=1.5, label='Command', alpha=0.7)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Velocity [m/s]')
    ax4.set_title('Velocity Profile')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # 5. Heading error vs time
    ax5 = plt.subplot(2, 3, 5)
    heading_error = np.degrees(data['theta_ref'] - data['theta'])
    # Normalize to [-180, 180]
    heading_error = (heading_error + 180) % 360 - 180
    ax5.plot(time, heading_error, 'm-', linewidth=1.5)
    ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Heading Error [deg]')
    ax5.set_title('Heading Error')
    ax5.grid(True, alpha=0.3)

    # 6. Path curvature vs arc length
    ax6 = plt.subplot(2, 3, 6)
    ax6.plot(data['s_ref'], data['kappa_ref'], 'b-', linewidth=1.5)
    ax6.set_xlabel('Arc Length [m]')
    ax6.set_ylabel('Curvature [1/m]')
    ax6.set_title('Path Curvature')
    ax6.grid(True, alpha=0.3)

    plt.tight_layout()

    # Save figure
    output_file = csv_file.replace('.csv', '.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    plt.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 plot_results.py <csv_file>")
        print("Example: python3 plot_results.py test_data/results/pid_figure8.csv")
        sys.exit(1)

    csv_file = sys.argv[1]
    plot_single_controller(csv_file)
