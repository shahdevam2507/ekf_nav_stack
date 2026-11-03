#!/usr/bin/env python3
"""
lidar_noise_characterizer.py
────────────────────────────
Point your LD19 at a flat static wall and run this node.
It collects range samples from a configurable angular window and
prints the noise statistics you need for R_sensor in the EKF.

Run:
    ros2 run calibrations lidar_noise

Output (example):
    ──────────────────────────────────────────
    LD19 Noise Characterization Results
    ──────────────────────────────────────────
    Samples collected : 1500
    Mean range        : 1.2341 m
    Std dev (sigma_r) : 0.0082 m   ← use as sqrt(R_sensor[0,0])
    Variance          : 0.0001 m²  ← R_sensor[0,0] directly
    Min / Max         : 1.212 / 1.258 m
    Angular res (est) : 0.0033 rad ← use as sqrt(R_sensor[1,1])

    ► Paste into ekfnode.py:
        R_sensor = np.diag([0.0001, 0.0000109, 0.0025])
                          [σ_r²    σ_φ²        σ_radius²]
    ──────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import yaml
import os
from collections import deque  # noqa: F401  (reserved for rolling buffer extensions)


class LidarNoiseCharacterizer(Node):
    def __init__(self):
        super().__init__('lidar_noise_characterizer')

        # ── Parameters ──────────────────────────────────────────────────────
        # Number of samples to collect before computing statistics
        self.declare_parameter('num_samples', 500)
        # Minimum valid range (filter out too-close readings)
        self.declare_parameter('min_range_m', 0.1)
        # Maximum valid range
        self.declare_parameter('max_range_m', 8.0)
        # Where to save the results YAML
        self.declare_parameter('output_path',
            '/home/jival/ekf_nav_stack/src/lidar_noise_results.yaml')

        self.num_samples = self.get_parameter('num_samples').value
        self.min_range   = self.get_parameter('min_range_m').value
        self.max_range   = self.get_parameter('max_range_m').value
        self.output_path = self.get_parameter('output_path').value

        # ── State ────────────────────────────────────────────────────────────
        self.range_samples = []    # valid range readings from all cropped beams
        self.scan_count    = 0
        self.done          = False

        # ── Subscriber ───────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info(
            f'Collecting {self.num_samples} samples from all valid /scan beams '
            f'(angle cropping handled by the LD19 launch file). '
            f'Point the LiDAR at a flat static wall.')

    def scan_callback(self, msg: LaserScan):
        if self.done:
            return

        # Accept all beams — angle cropping is already handled by ld19.launch.py.
        # Only filter out invalid (inf/nan) and out-of-range readings.
        for r in msg.ranges:
            if (not math.isinf(r) and not math.isnan(r)
                    and self.min_range < r < self.max_range):
                self.range_samples.append(r)

        self.scan_count += 1

        # Progress
        if self.scan_count % 20 == 0:
            self.get_logger().info(
                f'  {len(self.range_samples)} / {self.num_samples} samples...')

        if len(self.range_samples) >= self.num_samples:
            self.analyze_and_report(msg.angle_increment)

    def analyze_and_report(self, angle_increment_rad: float):
        self.done = True
        samples = np.array(self.range_samples[:self.num_samples])
        n       = len(samples)

        mean_r  = float(np.mean(samples))
        std_r   = float(np.std(samples))
        var_r   = float(np.var(samples))
        min_r   = float(np.min(samples))
        max_r   = float(np.max(samples))

        # Angular noise: LD19 angular resolution is the dominant source.
        # sigma_phi ≈ angle_increment / 2  (half a step, conservative estimate)
        sigma_phi = angle_increment_rad / 2.0
        var_phi   = sigma_phi ** 2

        # Radius (landmark size) noise — use std of range as a proxy
        # (you can characterize separately with cylindrical targets)
        sigma_radius = std_r * 2.0   # conservative: landmarks have more spread
        var_radius   = sigma_radius ** 2

        # ── Histogram ───────────────────────────────────────────────────────
        hist, bin_edges = np.histogram(samples, bins=20)
        bar_max = 30
        hist_norm = (hist / hist.max() * bar_max).astype(int)

        sep = '─' * 50
        self.get_logger().info('\n' + sep)
        self.get_logger().info('  LD19 Noise Characterization Results')
        self.get_logger().info(sep)
        self.get_logger().info(f'  Samples collected  : {n}')
        self.get_logger().info(f'  Mean range         : {mean_r:.4f} m')
        self.get_logger().info(f'  Std dev  (sigma_r) : {std_r:.4f} m')
        self.get_logger().info(f'  Variance (var_r)   : {var_r:.6f} m²')
        self.get_logger().info(f'  Min / Max          : {min_r:.3f} / {max_r:.3f} m')
        self.get_logger().info(f'  Angular resolution : {math.degrees(angle_increment_rad):.3f}°  '
                               f'→ sigma_phi ≈ {sigma_phi:.5f} rad')
        self.get_logger().info(sep)
        self.get_logger().info('  Range histogram:')
        for i in range(len(hist)):
            bar   = '█' * hist_norm[i]
            lo    = bin_edges[i]
            hi    = bin_edges[i + 1]
            self.get_logger().info(f'  {lo:.3f}-{hi:.3f} m | {bar} ({hist[i]})')
        self.get_logger().info(sep)
        self.get_logger().info('  ► Paste into ekfnode.py correction_step():')
        self.get_logger().info(
            f'      R_sensor = np.diag([{var_r:.6f}, {var_phi:.8f}, {var_radius:.6f}])')
        self.get_logger().info(
            f'                        #  σ_r²          σ_φ²              σ_radius²')
        self.get_logger().info(sep)

        # ── Save YAML ───────────────────────────────────────────────────────
        results = {
            'lidar_model': 'LD19',
            'samples': n,
            'range': {
                'mean_m'   : round(mean_r, 6),
                'std_m'    : round(std_r,  6),
                'var_m2'   : round(var_r,  8),
                'min_m'    : round(min_r,  4),
                'max_m'    : round(max_r,  4),
            },
            'angular': {
                'increment_rad' : round(angle_increment_rad, 6),
                'sigma_phi_rad' : round(sigma_phi,            6),
                'var_phi_rad2'  : round(var_phi,              8),
            },
            'radius_landmark': {
                'sigma_radius_m': round(sigma_radius, 6),
                'var_radius_m2' : round(var_radius,   8),
            },
            'R_sensor_diag': [
                round(var_r,      8),
                round(var_phi,    8),
                round(var_radius, 8),
            ],
            'note': (
                'R_sensor = np.diag(R_sensor_diag) — paste into correction_step(). '
                'Characterize at your typical operating range (0.5–3 m).'
            )
        }

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, 'w') as f:
            yaml.dump(results, f, default_flow_style=False)

        self.get_logger().info(f'  Results saved → {self.output_path}')
        self.get_logger().info(sep)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNoiseCharacterizer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

