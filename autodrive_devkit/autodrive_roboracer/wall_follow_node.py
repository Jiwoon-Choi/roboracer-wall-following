"""LiDAR left-wall follower for the AutoDRIVE RoboRacer practice track.

Uses two wall rays, geometric lookahead and PD steering. Throttle is reduced
for steering demand and limited front clearance. See TUNING.md for measured
experiments, fallback settings and validation limits.
"""

import math
import time
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


# Practice-track settings validated in simulator trials
DESIRED_DISTANCE = 0.8   # Distance from the left wall, in meters.
LOOKAHEAD = 0.8          # Predict wall clearance this far ahead.
KP = 1.0                # Correction for distance error.
KD = 0.1                # Correction for how quickly error changes.
BASE_THROTTLE = 0.18    # Normalized command, not meters per second.
STOP_DISTANCE = 0.5    # Stop requesting motion near a front obstacle.
SLOW_DISTANCE = 1.5    # Start reducing throttle below this clearance.
SCAN_TIMEOUT = 0.5     # Seconds without a scan before requesting a stop.


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        # Publishers send our commands to the bridge.
        self.throttle_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/throttle_command', 10
        )
        self.steering_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/steering_command', 10
        )

        # Call on_scan whenever a LiDAR message arrives.
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/roboracer_1/lidar',
            self.on_scan,
            qos_profile_sensor_data,
        )

        self.last_scan_time = None
        self.previous_time = None
        self.previous_error = None
        self.filtered_derivative = 0.0

        self.watchdog = self.create_timer(0.1, self.check_scan_timeout)
        self.get_logger().info('Wall follower ready. Waiting for LiDAR.')

    def publish_commands(self, throttle, steering):
        # Keep both commands inside the allowed range.
        throttle = max(-1.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))

        self.throttle_pub.publish(Float32(data=float(throttle)))
        self.steering_pub.publish(Float32(data=float(steering)))

    def stop(self):
        self.publish_commands(0.0, 0.0)
        self.previous_error = None
        self.previous_time = None
        self.filtered_derivative = 0.0

    def check_scan_timeout(self):
        if (
            self.last_scan_time is None
            or time.monotonic() - self.last_scan_time > SCAN_TIMEOUT
        ):
            self.stop()

    def get_wall_range(self, scan, angle_degrees):
        # Use a small group of rays around the requested angle.
        # The median reduces the influence of a single noisy reading.
        values = []

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if abs(angle - math.radians(angle_degrees)) <= math.radians(1.0):
                if (
                    math.isfinite(distance)
                    and scan.range_min <= distance < scan.range_max
                ):
                    values.append(distance)

        return median(values) if len(values) >= 3 else None

    def get_front_clearance(self, scan):
        values = []
        total = 0

        # Check a cone extending 15 degrees to either side of forward.
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if abs(angle) <= math.radians(15.0):
                total += 1

                # A below-minimum return is not evidence of clear space.
                # Request zero throttle rather than discard possible danger.
                if math.isfinite(distance) and 0.0 <= distance < scan.range_min:
                    return 0.0

                if distance == math.inf:
                    values.append(scan.range_max)
                elif (
                    math.isfinite(distance)
                    and scan.range_min <= distance <= scan.range_max
                ):
                    values.append(distance)

        # Do not drive using a mostly missing or invalid front scan.
        if total == 0 or len(values) < 0.8 * total:
            return None

        return min(values)

    def on_scan(self, scan):
        now = time.monotonic()
        self.last_scan_time = now

        if (
            not scan.ranges
            or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0.0
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or not 0.0 <= scan.range_min < scan.range_max
        ):
            self.stop()
            return

        left = self.get_wall_range(scan, 90.0)
        front_left = self.get_wall_range(scan, 45.0)
        front = self.get_front_clearance(scan)

        if left is None or front_left is None or front is None:
            self.stop()
            return

        if front <= STOP_DISTANCE:
            self.stop()
            return

        # Estimate the wall's angle from the two LiDAR rays.
        theta = math.radians(45.0)
        alpha = math.atan2(
            front_left * math.cos(theta) - left,
            front_left * math.sin(theta),
        )

        # Estimate perpendicular clearance now and slightly ahead.
        distance_now = left * math.cos(alpha)
        distance_ahead = distance_now + LOOKAHEAD * math.sin(alpha)

        # Positive error means too far from the left wall:
        # positive steering turns left, toward that wall.
        error = distance_ahead - DESIRED_DISTANCE

        derivative = 0.0
        if self.previous_error is not None:
            dt = now - self.previous_time
            if 0.001 <= dt <= SCAN_TIMEOUT:
                derivative = (error - self.previous_error) / dt

        # Smooth the derivative to reduce abrupt steering changes.
        self.filtered_derivative = (
            0.7 * self.filtered_derivative + 0.3 * derivative
        )

        steering = KP * error + KD * self.filtered_derivative
        steering = max(-1.0, min(1.0, steering))

        # Reduce throttle for strong turns and nearby front obstacles.
        turn_factor = 1.0 - 0.7 * abs(steering)
        front_factor = min(
            1.0,
            (front - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE),
        )
        throttle = BASE_THROTTLE * turn_factor * front_factor

        self.publish_commands(throttle, steering)
        self.previous_error = error
        self.previous_time = now


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()