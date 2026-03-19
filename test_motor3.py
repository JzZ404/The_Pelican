#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time, math

class Motor3Test(Node):
    def __init__(self):
        super().__init__('motor3_test')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/gix_controller/joint_trajectory',
            10
        )
        time.sleep(1.0)

    def move_to(self, position_rad, label, duration_sec=2):
        msg = JointTrajectory()
        msg.joint_names = ['gix']
        point = JointTrajectoryPoint()
        point.positions = [position_rad]
        point.time_from_start = Duration(sec=duration_sec)
        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info(f'Moving to {label} ({position_rad:.3f} rad)')
        time.sleep(duration_sec + 0.5)

def main():
    rclpy.init()
    node = Motor3Test()

    # move to initial position (45°)
    node.move_to(math.radians(45), 'init 20 deg')

    for i in range(2):
        print(f'--- Cycle {i+1}/2 ---')
        node.move_to(math.radians(145), '140 deg')
        node.move_to(math.radians(45), '20 deg')

    node.destroy_node()
    rclpy.shutdown()
    print('Done')
    
if __name__ == '__main__':
    main()