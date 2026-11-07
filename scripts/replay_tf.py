#!/usr/bin/env python3
"""Rebuild /clock and the map->base_link TF from /vehicle/odometry during bag replay.

Bags recorded without /tf and /clock cannot be shown in RViz (the path is in
base_link and all stamps are simulator time). Run this next to `ros2 bag play`
and start RViz with use_sim_time:=true.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class ReplayTf(Node):
    def __init__(self):
        super().__init__('replay_tf')
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.create_subscription(Odometry, '/vehicle/odometry', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, m: Odometry):
        self.clock_pub.publish(Clock(clock=m.header.stamp))
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = m.child_frame_id or 'base_link'
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.translation.z = m.pose.pose.position.z
        t.transform.rotation = m.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[t]))


def main():
    rclpy.init()
    rclpy.spin(ReplayTf())


if __name__ == '__main__':
    main()
