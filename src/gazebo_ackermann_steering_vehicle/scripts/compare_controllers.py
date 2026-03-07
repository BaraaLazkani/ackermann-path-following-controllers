#!/usr/bin/env python3

import sys
import os
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def load_controller_data(directory, trajectory='figure8'):
    """Load data for all controllers for a given trajectory"""
    controllers = ['pid', 'stanley', 'lqr', 'smc']
    data_dict = {}

    for controller in controllers:
        csv_file = os.path.join(directory, f'{controller}_{trajectory}.csv')
        if os.path.exists(csv_file):
            try:
                data_dict[controller] = pd.read_csv(csv_file)
                print(f"Loaded: {csv_file}")
            except Exception as e:
                print(f"Error loading {csv_file}: {e}")
        else:
            print(f"Warning: {csv_file} not found, skipping...")

    return data_dict


def compute_metrics(data):
    """Compute performance metrics from data"""
    metrics = {}
    metrics['mean_cte'] = np.mean(np.abs(data['cte']))
    metrics['rms_cte'] = np.sqrt(np.mean(data['cte']**2))
    metrics['max_cte'] = np.max(np.abs(data['cte']))
    metrics['mean_steering'] = np.mean(np.abs(data['steering_cmd']))
    metrics['max_steering'] = np.max(np.abs(data['steering_cmd']))
    metrics['num_samples'] = len(data)
    return metrics


def plot_comparison(data_dict, trajectory_name):
    """Create comparison plots for all controllers"""

    if not data_dict:
        print("No data to plot!")
        return

    # Color scheme for controllers
    colors = {
        'pid': 'blue',
        'stanley': 'green',
        'lqr': 'red',
        'smc': 'purple'
    }

    labels = {
        'pid': 'PID',
        'stanley': 'Stanley',
        'lqr': 'LQR',
        'smc': 'Sliding Mode'
    }

    # Create figure
    fig = plt.figure(figsize=(16, 12))

    # 1. Trajectory comparison
    ax1 = plt.subplot(2, 3, 1)
    # Plot reference path (same for all)
    first_controller = list(data_dict.keys())[0]
    ax1.plot(data_dict[first_controller]['x_ref'],
             data_dict[first_controller]['y_ref'],
             'k--', linewidth=2.5, label='Reference', alpha=0.5, zorder=1)

    for controller, data in data_dict.items():
        ax1.plot(data['x'], data['y'], color=colors[controller],
                linewidth=1.5, label=labels[controller], alpha=0.8)

    ax1.set_xlabel('X Position [m]', fontsize=11)
    ax1.set_ylabel('Y Position [m]', fontsize=11)
    ax1.set_title('Trajectory Comparison', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # 2. Cross-track error comparison
    ax2 = plt.subplot(2, 3, 2)
    for controller, data in data_dict.items():
        time = data['timestamp'] - data['timestamp'].iloc[0]
        ax2.plot(time, data['cte'], color=colors[controller],
                linewidth=1.5, label=labels[controller], alpha=0.8)

    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time [s]', fontsize=11)
    ax2.set_ylabel('Cross-Track Error [m]', fontsize=11)
    ax2.set_title('Cross-Track Error Comparison', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)

    # 3. Steering angle comparison
    ax3 = plt.subplot(2, 3, 3)
    for controller, data in data_dict.items():
        time = data['timestamp'] - data['timestamp'].iloc[0]
        ax3.plot(time, np.degrees(data['steering_cmd']), color=colors[controller],
                linewidth=1.5, label=labels[controller], alpha=0.8)

    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_xlabel('Time [s]', fontsize=11)
    ax3.set_ylabel('Steering Angle [deg]', fontsize=11)
    ax3.set_title('Steering Command Comparison', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=10)
    ax3.grid(True, alpha=0.3)

    # 4. CTE histogram
    ax4 = plt.subplot(2, 3, 4)
    bins = np.linspace(-0.5, 0.5, 50)
    for controller, data in data_dict.items():
        ax4.hist(data['cte'], bins=bins, alpha=0.5, label=labels[controller],
                color=colors[controller], edgecolor='black')

    ax4.set_xlabel('Cross-Track Error [m]', fontsize=11)
    ax4.set_ylabel('Frequency', fontsize=11)
    ax4.set_title('CTE Distribution', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=10)
    ax4.grid(True, alpha=0.3, axis='y')

    # 5. Metrics comparison table
    ax5 = plt.subplot(2, 3, 5)
    ax5.axis('off')

    # Compute metrics for all controllers
    all_metrics = {}
    for controller, data in data_dict.items():
        all_metrics[controller] = compute_metrics(data)

    # Create table data
    table_data = []
    table_data.append(['Metric', 'PID', 'Stanley', 'LQR', 'SMC'])
    table_data.append(['-'*15, '-'*10, '-'*10, '-'*10, '-'*10])

    metrics_to_show = [
        ('Mean |CTE| [m]', 'mean_cte', '{:.4f}'),
        ('RMS CTE [m]', 'rms_cte', '{:.4f}'),
        ('Max |CTE| [m]', 'max_cte', '{:.4f}'),
        ('Mean |δ| [deg]', 'mean_steering', '{:.2f}', lambda x: np.degrees(x)),
        ('Max |δ| [deg]', 'max_steering', '{:.2f}', lambda x: np.degrees(x)),
    ]

    for metric_name, metric_key, fmt, *transform in metrics_to_show:
        row = [metric_name]
        for controller in ['pid', 'stanley', 'lqr', 'smc']:
            if controller in all_metrics:
                value = all_metrics[controller][metric_key]
                if transform:
                    value = transform[0](value)
                row.append(fmt.format(value))
            else:
                row.append('N/A')
        table_data.append(row)

    # Display table
    table_text = '\n'.join([' | '.join(row) for row in table_data])
    ax5.text(0.1, 0.9, table_text, transform=ax5.transAxes,
            fontsize=10, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    ax5.set_title('Performance Metrics', fontsize=12, fontweight='bold', pad=20)

    # 6. Bar chart comparison
    ax6 = plt.subplot(2, 3, 6)
    controllers_list = list(all_metrics.keys())
    x_pos = np.arange(len(controllers_list))
    mean_ctes = [all_metrics[c]['mean_cte'] for c in controllers_list]
    bar_colors = [colors[c] for c in controllers_list]

    bars = ax6.bar(x_pos, mean_ctes, color=bar_colors, alpha=0.7, edgecolor='black')
    ax6.set_xlabel('Controller', fontsize=11)
    ax6.set_ylabel('Mean |CTE| [m]', fontsize=11)
    ax6.set_title('Mean Cross-Track Error Comparison', fontsize=12, fontweight='bold')
    ax6.set_xticks(x_pos)
    ax6.set_xticklabels([labels[c] for c in controllers_list], fontsize=10)
    ax6.grid(True, alpha=0.3, axis='y')

    # Add value labels on bars
    for bar in bars:
        height = bar.get_height()
        ax6.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.4f}',
                ha='center', va='bottom', fontsize=9)

    plt.suptitle(f'Controller Comparison - {trajectory_name.upper()} Trajectory',
                fontsize=14, fontweight='bold')
    plt.tight_layout()

    # Save figure
    output_file = f'test_data/results/comparison_{trajectory_name}.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nComparison plot saved to: {output_file}")
    plt.close()


def print_summary_table(data_dict):
    """Print a summary table of metrics"""
    print("\n" + "="*80)
    print("CONTROLLER COMPARISON SUMMARY")
    print("="*80)

    # Compute metrics for all controllers
    all_metrics = {}
    for controller, data in data_dict.items():
        all_metrics[controller] = compute_metrics(data)

    # Print header
    print(f"{'Metric':<25} {'PID':>12} {'Stanley':>12} {'LQR':>12} {'SMC':>12}")
    print("-"*80)

    # Print metrics
    metrics_to_show = [
        ('Mean |CTE| [m]', 'mean_cte', '{:>12.4f}'),
        ('RMS CTE [m]', 'rms_cte', '{:>12.4f}'),
        ('Max |CTE| [m]', 'max_cte', '{:>12.4f}'),
        ('Mean |Steering| [deg]', 'mean_steering', '{:>12.2f}', lambda x: np.degrees(x)),
        ('Max |Steering| [deg]', 'max_steering', '{:>12.2f}', lambda x: np.degrees(x)),
        ('Samples', 'num_samples', '{:>12d}'),
    ]

    for metric_name, metric_key, fmt, *transform in metrics_to_show:
        row = f"{metric_name:<25}"
        for controller in ['pid', 'stanley', 'lqr', 'smc']:
            if controller in all_metrics:
                value = all_metrics[controller][metric_key]
                if transform:
                    value = transform[0](value)
                row += " " + fmt.format(value)
            else:
                row += f"{'N/A':>12}"
        print(row)

    print("="*80 + "\n")


if __name__ == "__main__":
    # Default settings
    data_dir = "test_data/results"
    trajectory = "figure8"

    # Parse command line arguments
    if len(sys.argv) >= 2:
        data_dir = sys.argv[1]
    if len(sys.argv) >= 3:
        trajectory = sys.argv[2]

    print(f"Loading controller data from: {data_dir}")
    print(f"Trajectory: {trajectory}\n")

    # Load data
    data_dict = load_controller_data(data_dir, trajectory)

    if not data_dict:
        print("No data files found. Please run controllers first.")
        sys.exit(1)

    # Print summary table
    print_summary_table(data_dict)

    # Create comparison plots
    plot_comparison(data_dict, trajectory)
