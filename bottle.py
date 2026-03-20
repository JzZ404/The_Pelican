#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math, threading, time, random

IMAGE_WIDTH    = 320
IMAGE_HEIGHT   = 240
IMAGE_CENTER_X = IMAGE_WIDTH  / 2.0
IMAGE_CENTER_Y = IMAGE_HEIGHT / 2.0
KP_ANG = 0.012
KI_ANG = 0.0001
KD_ANG = 0.004
KP_LIN = 0.01
KI_LIN = 0.0001
KD_LIN = 0.002
DEADBAND_ANG = 20.0
DEADBAND_LIN = 20.0
MAX_ANG_VEL  = 0.8
MAX_LIN_VEL  = 0.25
OBSTACLE_DIST     = 0.5
WANDER_LIN_VEL    = 0.12
WANDER_ANG_VEL    = 0.5
NO_DETECT_TIMEOUT = 7.0
STATE_WANDER = 0
STATE_BACKUP = 1
STATE_TURN   = 2
STATE_TRACK  = 3

class BottleAlignNode(Node):
    def __init__(self):
        super().__init__('bottle_align_node')
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(DetectionArray, '/yolo/detections', self.detection_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, scan_qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_pub = self.create_publisher(JointTrajectory, '/gix_controller/joint_trajectory', 10)
        self.prev_error_ang = 0.0
        self.integral_ang   = 0.0
        self.prev_error_lin = 0.0
        self.integral_lin   = 0.0
        self.prev_time = self.get_clock().now()
        self.centered_since  = None
        self.motor_triggered = False
        self.motor_running   = False
        self.latest_scan     = None
        self.last_detect_time = None
        self.state       = STATE_WANDER
        self.state_until = 0.0
        self.turn_dir    = 1.0
        self.create_timer(0.05, self.wander_tick)
        self.get_logger().info('Bottle align node started — WANDERING')

    def scan_cb(self, msg):
        self.latest_scan = msg

    def _sector(self, center_deg, half_deg):
        if self.latest_scan is None:
            return float('inf')
        ranges = self.latest_scan.ranges
        n   = len(ranges)
        inc = self.latest_scan.angle_increment
        rmin = self.latest_scan.range_min
        rmax = self.latest_scan.range_max
        c = int(math.radians(center_deg) / inc) % n
        w = int(math.radians(half_deg) / inc)
        idxs = [(c + d) % n for d in range(-w, w + 1)]
        vals = [ranges[i] for i in idxs if rmin < ranges[i] < rmax]
        return min(vals) if vals else float('inf')

    def wander_tick(self):
        if self.motor_running:
            return
        now = time.time()
        if self.last_detect_time is not None and (now - self.last_detect_time) < NO_DETECT_TIMEOUT:
            if self.state != STATE_TRACK:
                self.get_logger().info('Object detected — switching to TRACK')
                self.state = STATE_TRACK
            return
        if self.state == STATE_TRACK:
            self.get_logger().info('No object for 7s — resuming WANDER')
            self.state = STATE_WANDER
        front = self._sector(180, 60)
        self.get_logger().info(f'WANDER front={front:.2f}m state={self.state}', throttle_duration_sec=1.0)
        twist = Twist()
        if self.state == STATE_WANDER:
            if front < OBSTACLE_DIST:
                self.state = STATE_BACKUP
                self.state_until = now + 1.5
                self.turn_dir = 1.0 if random.random() > 0.5 else -1.0
                self.get_logger().info('OBSTACLE — backing up')
            else:
                twist.linear.x = -WANDER_LIN_VEL
        elif self.state == STATE_BACKUP:
            if now < self.state_until:
                twist.linear.x = WANDER_LIN_VEL
            else:
                self.state = STATE_TURN
                self.state_until = now + 3.0
        elif self.state == STATE_TURN:
            if now < self.state_until:
                twist.angular.z = WANDER_ANG_VEL * self.turn_dir
            else:
                self.state = STATE_WANDER
        self.pub.publish(twist)

    def detection_cb(self, msg):
        bottle = None
        best_score = 0.0
        for det in msg.detections:
            if det.class_name in ['bottle', 'wine glass', 'cup', 'skateboard'] and det.score > best_score:
                bottle = det
                best_score = det.score
        if bottle is None:
            self._reset_pid()
            return
        self.last_detect_time = time.time()
        if self.motor_running:
            self.pub.publish(Twist())
            return
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        if dt <= 0:
            return
        bottle_x = bottle.bbox.center.position.x
        bottle_y = bottle.bbox.center.position.y
        error_x = -(IMAGE_CENTER_X - bottle_x)
        error_y = -(IMAGE_CENTER_Y - bottle_y)
        self.integral_ang  += error_x * dt
        derivative_ang      = (error_x - self.prev_error_ang) / dt
        self.prev_error_ang = error_x
        ang_vel = KP_ANG * error_x + KI_ANG * self.integral_ang + KD_ANG * derivative_ang
        ang_vel = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, ang_vel))
        if abs(error_x) < DEADBAND_ANG:
            ang_vel = 0.0
        self.integral_lin  += error_y * dt
        derivative_lin      = (error_y - self.prev_error_lin) / dt
        self.prev_error_lin = error_y
        lin_vel = KP_LIN * error_y + KI_LIN * self.integral_lin + KD_LIN * derivative_lin
        lin_vel = max(-MAX_LIN_VEL, min(MAX_LIN_VEL, lin_vel))
        align_factor = max(0.0, 1.0 - abs(error_x) / (IMAGE_WIDTH / 2.0))
        lin_vel *= align_factor
        twist = Twist()
        twist.angular.z = ang_vel
        if abs(error_y) < DEADBAND_LIN and abs(error_x) < DEADBAND_ANG:
            lin_vel = 0.0
            now_sec = now.nanoseconds / 1e9
            if self.centered_since is None:
                self.centered_since = now_sec
                self.motor_triggered = False
            elapsed = now_sec - self.centered_since
            self.get_logger().info(f'FULLY ALIGNED! ({elapsed:.1f}/0.003s)', throttle_duration_sec=0.3)
            if elapsed >= 0.003 and not self.motor_triggered:
                self.motor_triggered = True
                self._trigger_motor3()
        else:
            self.centered_since = None
            self.get_logger().info(f'ex={error_x:.0f}px ang={ang_vel:.2f}  ey={error_y:.0f}px lin={lin_vel:.2f}', throttle_duration_sec=0.3)
        twist.linear.x = lin_vel
        self.pub.publish(twist)

    def _trigger_motor3(self):
        self.get_logger().info('Triggering motor 3!')
        self.motor_running = True
        threading.Thread(target=self._run_motor_sequence, daemon=True).start()

    def _run_motor_sequence(self):
        def send(position_rad, duration_sec=2):
            msg = JointTrajectory()
            msg.joint_names = ['gix']
            pt = JointTrajectoryPoint()
            pt.positions = [position_rad]
            pt.time_from_start = Duration(sec=duration_sec)
            msg.points = [pt]
            self.motor_pub.publish(msg)
            time.sleep(duration_sec + 0.5)
        send(math.radians(45), duration_sec=1)
        for _ in range(1):
            send(math.radians(145), duration_sec=1)
            time.sleep(3.0)
            send(math.radians(45), duration_sec=1)
            time.sleep(5.0)
        self.motor_running = False

    def _reset_pid(self):
        self.prev_error_ang = 0.0
        self.integral_ang   = 0.0
        self.prev_error_lin = 0.0
        self.integral_lin   = 0.0
        self.centered_since = None

def main():
    rclpy.init()
    node = BottleAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
