#!/usr/bin/env python3
"""
Visualize ORB-SLAM3 trajectory from TUM format file
Usage: python3 plot_trajectory.py [trajectory_file.txt]
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for Docker
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os


def load_tum_trajectory(filename):
    """Load trajectory in TUM format: timestamp tx ty tz qx qy qz qw"""
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quaternions = data[:, 4:8]
    return timestamps, positions, quaternions


def plot_trajectory_2d(positions, output_file='trajectory_2d.png'):
    """Plot 2D trajectory views"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    # XY plot (top-down)
    axes[0].plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1)
    axes[0].scatter(positions[0, 0], positions[0, 1], c='g', s=100, marker='o', label='Start')
    axes[0].scatter(positions[-1, 0], positions[-1, 1], c='r', s=100, marker='x', label='End')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('Top-Down View (XY)')
    axes[0].legend()
    axes[0].axis('equal')
    axes[0].grid(True)
    
    # XZ plot (side view)
    axes[1].plot(positions[:, 0], positions[:, 2], 'b-', linewidth=1)
    axes[1].scatter(positions[0, 0], positions[0, 2], c='g', s=100, marker='o', label='Start')
    axes[1].scatter(positions[-1, 0], positions[-1, 2], c='r', s=100, marker='x', label='End')
    axes[1].set_xlabel('X (m)')
    axes[1].set_ylabel('Z (m)')
    axes[1].set_title('Side View (XZ)')
    axes[1].legend()
    axes[1].axis('equal')
    axes[1].grid(True)
    
    # YZ plot (front view)
    axes[2].plot(positions[:, 1], positions[:, 2], 'b-', linewidth=1)
    axes[2].scatter(positions[0, 1], positions[0, 2], c='g', s=100, marker='o', label='Start')
    axes[2].scatter(positions[-1, 1], positions[-1, 2], c='r', s=100, marker='x', label='End')
    axes[2].set_xlabel('Y (m)')
    axes[2].set_ylabel('Z (m)')
    axes[2].set_title('Front View (YZ)')
    axes[2].legend()
    axes[2].axis('equal')
    axes[2].grid(True)
    
    plt.suptitle('ORB-SLAM3 Trajectory')
    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    print(f"Saved {output_file}")
    return fig


def plot_trajectory_3d(positions, output_file='trajectory_3d.png'):
    """Plot 3D trajectory"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=1, label='Trajectory')
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c='g', s=100, marker='o', label='Start')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='r', s=100, marker='x', label='End')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('ORB-SLAM3 3D Trajectory')
    ax.legend()
    
    # Make axes equal
    max_range = np.array([
        positions[:, 0].max() - positions[:, 0].min(),
        positions[:, 1].max() - positions[:, 1].min(),
        positions[:, 2].max() - positions[:, 2].min()
    ]).max() / 2.0
    
    mid_x = (positions[:, 0].max() + positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max() + positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max() + positions[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.savefig(output_file, dpi=150)
    print(f"Saved {output_file}")
    return fig


def print_statistics(positions, timestamps):
    """Print trajectory statistics"""
    total_distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
    duration = (timestamps[-1] - timestamps[0]) / 1e9 if timestamps[-1] > 1e6 else timestamps[-1] - timestamps[0]
    
    print("\n" + "="*50)
    print("TRAJECTORY STATISTICS")
    print("="*50)
    print(f"Number of poses: {len(positions)}")
    print(f"Total distance traveled: {total_distance:.3f} m")
    print(f"Duration: {duration:.2f} s")
    print(f"X range: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m")
    print(f"Y range: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m")
    print(f"Z range: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m")
    print("="*50)


def main():
    # Default file or command line argument
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        # Try common locations
        candidates = [
            "CameraTrajectory.txt",
            "KeyFrameTrajectory.txt",
            "/tmp/orb_slam3_trajectory.txt",
            "/tmp/orb_slam3_camera_trajectory.txt"
        ]
        filename = None
        for f in candidates:
            if os.path.exists(f):
                filename = f
                break
        
        if filename is None:
            print("Usage: python3 plot_trajectory.py [trajectory_file.txt]")
            print("\nNo trajectory file found. Looked for:")
            for f in candidates:
                print(f"  - {f}")
            return
    
    print(f"Loading trajectory from: {filename}")
    
    try:
        timestamps, positions, quaternions = load_tum_trajectory(filename)
    except Exception as e:
        print(f"Error loading file: {e}")
        return
    
    print_statistics(positions, timestamps)
    
    # Determine output directory
    output_dir = os.path.dirname(filename) if os.path.dirname(filename) else "."
    
    # Plot 2D views
    plot_trajectory_2d(positions, os.path.join(output_dir, 'trajectory_2d.png'))
    
    # Plot 3D view
    plot_trajectory_3d(positions, os.path.join(output_dir, 'trajectory_3d.png'))
    
    print("\nVisualization complete!")


if __name__ == "__main__":
    main()
