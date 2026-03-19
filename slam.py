#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math, time, random

OBSTACLE_DIST = 0.5
WANDER_LIN    = -0.15
WANDER_ANG    = 1.2

STATE_FORWARD = 0
STATE_BACKUP  = 1
STATE_TURN    = 2

class SlamWander(Node):
    def __init__(self):
        super().__init__('slam_wander')
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, scan_qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_scan = None
        self.state = STATE_FORWARD
        self.state_until = 0.0
        self.turn_dir = 1.0
        self.create_timer(0.05, self.tick)
        self.get_logger().info('SLAM wander started')

    def scan_cb(self, msg):
        self.latest_scan = msg

    def tick(self):
        twist = Twist()
        if self.latest_scan is None:
            twist.linear.x = WANDER_LIN
            self.pub.publish(twist)
            return

        ranges = self.latest_scan.ranges
        n   = len(ranges)
        inc = self.latest_scan.angle_increment
        rmin = self.latest_scan.range_min
        rmax = self.latest_scan.range_max

        def sector(center_deg, half_deg):
            c = int(math.radians(center_deg) / inc) % n
            w = int(math.radians(half_deg) / inc)
            idxs = [(c + d) % n for d in range(-w, w + 1)]
            vals = [ranges[i] for i in idxs if rmin < ranges[i] < rmax]
            return min(vals) if vals else float('inf')

        front = sector(180, 60)
        self.get_logger().info(f'front={front:.2f}m state={self.state}', throttle_duration_sec=0.5)

        now = time.time()

        if self.state == STATE_FORWARD:
            if front < OBSTACLE_DIST:
                self.state = STATE_BACKUP
                self.state_until = now + 1.5
                self.turn_dir = 1.0 if random.random() > 0.5 else -1.0
                self.get_logger().info('OBSTACLE — backing up')
            else:
                twist.linear.x = WANDER_LIN

        elif self.state == STATE_BACKUP:
            if now < self.state_until:
                twist.linear.x = 0.15
            else:
                self.state = STATE_TURN
                self.state_until = now + 3.0

        elif self.state == STATE_TURN:
            if now < self.state_until:
                twist.angular.z = WANDER_ANG * self.turn_dir
            else:
                self.state = STATE_FORWARD

        self.pub.publish(twist)

def main():
    rclpy.init()
    node = SlamWander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
