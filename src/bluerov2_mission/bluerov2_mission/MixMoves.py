#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from mavros_msgs.msg import ManualControl


class ManualControlMixer(Node):
    def __init__(self):
        super().__init__('manual_control_mixer')

        # Latest commanded values from each source, defaulting to safe/neutral
        self.surge = 0.0
        self.sway = 0.0
        self.yaw = 0.0
        self.depth_thrust = 0.0

        self.create_subscription(Float64MultiArray, '/cmd/surge_sway', self.surge_sway_cb, 10)
        self.create_subscription(Float64, '/cmd/yaw', self.yaw_cb, 10)
        self.create_subscription(Float64, '/cmd/depth_thrust', self.depth_cb, 10)

        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub_rov1 = self.create_publisher(ManualControl, '/rov1/manual_control', 10)

        # Publish combined command at a fixed rate, independent of when
        # individual setpoints arrive
        self.create_timer(0.1, self.publish_combined)

        self.get_logger().info("ManualControlMixer node initialized")

    def surge_sway_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self.surge = msg.data[0]
            self.sway = msg.data[1]

    def yaw_cb(self, msg: Float64):
        self.yaw = msg.data

    def depth_cb(self, msg: Float64):
        self.depth_thrust = msg.data

    def publish_combined(self):
        mc = ManualControl()
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.x = float(self.surge)
        mc.y = float(self.sway)
        mc.z = float(self.depth_thrust)
        mc.r = float(self.yaw)
        self.pub.publish(mc)
        self.pub_rov1.publish(mc)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()