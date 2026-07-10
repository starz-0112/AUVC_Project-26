#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Polygon

class FakeBattery(Node):
    def __init__(self):
        super().__init__('fake_battery_node')

        #Publish
        self.pub = self.create_publisher(Int32, '/fake_battery', 10)
        self.battery_level = 100

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.dead = False

        self.get_logger().info("FakeBattery node started, current level 100")

        self.target_sub = self.create_subscription(Polygon, '/current_target', self.target_callback, 10)
        self.dock = (0, 0.0, 0.0, 0.0) #Make sure this matches coords from PathPlanning
        self.last_target = None

    def target_callback(self, msg):
        if len(msg.points) == 0:
            return

        p = msg.points[0]
        current_target = (p.x, p.y, p.z)

        if self.last_target == self.dock and current_target != self.dock: 
            self.battery_level = 100
            self.dead = False
            self.get_logger().info("Battery reset to 100%")

        self.last_target = current_target
    
    def timer_callback(self):
        if self.dead:
            self.get_logger().info("CRITICAL FAILURE: BATTERY 0")
            return
        
        self.battery_level -= 5

        if self.battery_level <= 0:
            self.battery_level = 0
            self.dead = True

            msg = Int32()
            msg.data = self.battery_level
            self.pub.publish(msg)

            self.get_logger().error("CRITICAL FAILURE: BATTERY 0")
            return

        msg = Int32()
        msg.data = self.battery_level
        self.pub.publish(msg)

        self.get_logger().info(f"Battery level: {self.battery_level}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeBattery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()