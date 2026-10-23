#!/usr/bin/env python3
"""
Lab 3: Wall Following (LEFT / inner wall, counter-clockwise around Levine).
Reads /scan, publishes /drive.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        # ---------------- tuned defaults ----------------
        p = self.declare_parameter
        self.kp = p('kp', 1.0).value
        self.ki = p('ki', 0.0).value
        self.kd = p('kd', 0.1).value
        self.desired_dist = p('desired_dist', 1.0).value     # m from left wall
        self.lookahead = p('lookahead', 1.0).value           # L, metres
        self.theta_deg = p('theta_deg', 50.0).value          # angle between beams a and b
        self.max_steer = p('max_steer', 0.4189).value        # rad (~24 deg)
        self.integral_limit = p('integral_limit', 1.0).value

        # doorway rejection
        self.jump_thresh = p('jump_thresh', 0.3).value       # m, sudden jump in projected dist
        self.front_clear = p('front_clear', 3.5).value       # m, "no wall ahead" = doorway
        self.max_hold = p('max_hold', 80).value              # scans to ignore a jump (~2 s)

        # speeds (m/s)
        self.speed_fast = p('speed_fast', 1.5).value
        self.speed_mid = p('speed_mid', 1.0).value
        self.speed_slow = p('speed_slow', 0.5).value
        # -----------------------------------------------

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        self.last_good = None
        self.hold_count = 0

    @staticmethod
    def _valid(scan, r):
        return math.isfinite(r) and scan.range_min < r < scan.range_max

    def get_range(self, scan, angle_deg, half_window=2):
        """Average range around angle_deg (0 = ahead, + = left), skipping inf/NaN."""
        angle = math.radians(angle_deg)
        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        lo = max(0, idx - half_window)
        hi = min(len(scan.ranges), idx + half_window + 1)
        vals = [r for r in scan.ranges[lo:hi] if self._valid(scan, r)]
        return sum(vals) / len(vals) if vals else scan.range_max

    def front_distance(self, scan, half_angle_deg=5.0):
        """Closest obstacle in a narrow cone straight ahead."""
        lo = int((math.radians(-half_angle_deg) - scan.angle_min) / scan.angle_increment)
        hi = int((math.radians(half_angle_deg) - scan.angle_min) / scan.angle_increment)
        lo, hi = max(0, lo), min(len(scan.ranges), hi + 1)
        vals = [r for r in scan.ranges[lo:hi] if self._valid(scan, r)]
        return min(vals) if vals else scan.range_max

    def scan_callback(self, scan):
        # 1. two beams on the LEFT wall
        theta = math.radians(self.theta_deg)
        b = self.get_range(scan, 90.0)
        a = self.get_range(scan, 90.0 - self.theta_deg)

        # 2-4. angle to wall, current distance, projected distance
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        d_now = b * math.cos(alpha)
        d_future = d_now + self.lookahead * math.sin(alpha)

        # doorway vs corner: corner has a wall ahead, doorway does not
        front = self.front_distance(scan)
        jumped = self.last_good is not None and (d_future - self.last_good) > self.jump_thresh
        if jumped and front > self.front_clear and self.hold_count < self.max_hold:
            d_future = self.last_good
            self.hold_count += 1
        else:
            self.last_good = d_future
            self.hold_count = 0

        # 5. PID (too far from left wall -> positive error -> steer left)
        error = d_future - self.desired_dist

        t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        dt = (t - self.prev_time) if self.prev_time is not None else 0.0
        if dt <= 0.0 or dt > 0.5:
            dt = 0.025
        self.prev_time = t

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        steer = self.kp * error + self.ki * self.integral + self.kd * derivative
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # 6. speed from steering angle
        steer_deg = abs(math.degrees(steer))
        if steer_deg < 10.0:
            speed = self.speed_fast
        elif steer_deg < 20.0:
            speed = self.speed_mid
        else:
            speed = self.speed_slow
        if front < 1.0:
            speed = min(speed, self.speed_slow)

        # 7. publish
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
