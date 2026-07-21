#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, Float64, Float64MultiArray #matches depth
from geometry_msgs.msg import Point32 #waypoint/flashylights stuff
from mavros_msgs.msg import ManualControl # movement

import math

## SEE THAT CURRENT COORDINATES MUST HAVE DEPTH IN METERS!!! Actually, everything in meters T-T

class ROVMove(Node):
    def __init__(self):
        super().__init__("ROV_move")

        #List of all topics subscribed to
        self.create_subscription(Float64, '/depth', self.depth_cb, 10) #depth
        # self.create_subscription(Imu, '/imu', self.imu_cb, 10) #heading
        self.create_subscription(Float64MultiArray, '/current_target', self.target_cb, 10) #current target

        self.create_subscription(Point32, "/current_position", self.position_cb, 10)
        
        self.create_subscription(Int16, "/heading", self.heading_cb, 10)
        self.create_subscription(Int16, "/rov1/heading", self.heading_cb, 10)

        #Publish (movement control)
        # self.pub_manual = self.create_publisher(ManualControl, '/manual_control', 10)
        # self.pub_manual = self.create_publisher(ManualControl, '/rov1/manual_control', 10)
        self.pub_surge_sway = self.create_publisher(Float64MultiArray, '/cmd/surge_sway', 10)

        self.pub_depth_setpoint = self.create_publisher(Float64, '/target_depth', 10)
        self.pub_heading_setpoint = self.create_publisher(Float64, '/target_heading', 10)

        #State
        self.current_target = None
        self.heading_offset = None

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.depth = None
        self.yaw = 0.0

        #movement parameter gimmicks
        self.forward_speed = 100
        self.yaw_speed = 100

        #time loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("ROVMove node initialized")

    def position_cb(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
    
    def publish_manual(self, x, y, r):
        msg = Float64MultiArray()
        msg.data = [float(x), float(y)]
        self.pub_surge_sway.publish(msg)
    
    #Data callbacks
    def target_cb(self, msg: Float64MultiArray):

        if len(msg.data) < 3:
            self.get_logger().warn("Received target with fewer than 3 values.")
            return

        self.current_target = (
            msg.data[0],
            msg.data[1],
            msg.data[2]
        )   

        self.get_logger().info(f"New target set: {self.current_target}")

        # Publish depth setpoint
        depth_msg = Float64()
        depth_msg.data = msg.data[2]
        self.pub_depth_setpoint.publish(depth_msg)

    def heading_cb(self, msg: Int16):
        raw = float(msg.data)

        # Only initialize once
        if self.heading_offset is None:
            if raw <= 90.0:
                self.heading_offset = 90.0 - raw
                self.yaw = (raw + self.heading_offset) % 360
            elif raw > 90.0:
                self.heading_offset = raw - 90.0
                self.yaw = (raw - self.heading_offset) % 360
            self.get_logger().info(
                f"Heading offset initialized: {self.heading_offset:.1f}° "
                f"(raw={raw:.1f}°)"
            )
    
    # def imu_cb(self, msg: Imu):
        # "convert quaternion → yaw"
        # self.yaw = self.quat_to_yaw(msg.orientation)

    def depth_cb(self, msg: Float64):
        self.depth = msg.data

    # Main controls
    def control_loop(self):
        if self.current_target is None:
            return
    
        tx, ty, tz = self.current_target

        #computes relative vector from assumed pos
        dx = tx - self.robot_x
        dy = ty - self.robot_y

        self.get_logger().info(
            f"Robot=({self.robot_x:.2f}, {self.robot_y:.2f}) "
            f"Target=({tx:.2f}, {ty:.2f}) "
            f"dx={dx:.2f} dy={dy:.2f}"
        )

        #computes heading so it faces the next target
        desired_heading = math.degrees(math.atan2(dx, dy)) % 360

        #publish heading out to PID controller
        msg = Float64()
        msg.data = desired_heading
        self.pub_heading_setpoint.publish(msg)

        #computes heading error
        heading_error = abs(self.angle_diff(desired_heading, self.yaw))
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.2:
            surge = 0
        elif distance < 0.6:
            surge = 50
        elif distance < 1.5:
            surge = 100
        
        if heading_error > 45:
            surge = 0
        else:
            surge = self.forward_speed

        self.publish_manual(surge, 0.0, 0.0)

    # def quat_to_yaw(self, q):
        # x, y, z, w = q.x, q.y, q.z, q.w
        # siny = 2.0 * (w*z + x*y)
        # cosy = 1.0 - 2.0 * (y*y + z*z)
        # return math.degrees(math.atan2(siny, cosy)) % 360

    def angle_diff(self, target, current):
        return ((target - current + 540) % 360) - 180

def main(args=None):
    rclpy.init(args=args)
    node = ROVMove()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()