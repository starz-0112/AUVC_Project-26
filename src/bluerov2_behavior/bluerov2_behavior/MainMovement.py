import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64 #matches depth
from sensor_msgs.msg import Imu #heading?
from geometry_msgs.msg import Point32 #waypoint/flashylights stuff
from mavros_msgs.msg import ManualControl # movement

#Probably not used but hey here if needed
from mavros_msgs.msg import OverrideRCIn

import math
import numpy as np

## SEE THAT CURRENT COORDINATES MUST HAVE DEPTH IN METERS!!!

class ROVMove(Node):
    def __init__(self):
        super().__init__("ROV_move")

        #List of all topics subscribed to
        self.create_subscription(Float64, "/depth", self.depth_cb, 10) #depth
        self.create_subscription(Imu, "/imu", self.imu_cb, 10) #heading
        self.create_subscription(Point32, "/current_target", self.target_cb, 10) #current target

        #Publish (movement control)
        self.pub_manual = self.create_publisher(ManualControl, "/manual_control", 10)
        self.pub_depth_setpoint = self.create_publisher(Float64, "/target_depth", 10)
        self.pub_heading_setpoint = self.create_publisher(Float64, "/target_heading", 10)

        #State
        self.current_target = None
        self.last_target = None

        self.assumed_x = 0.0 #START, ADJUST LATER TO BE DOCK
        self.assumed_y = 0.0

        self.depth = None
        self.yaw = None

        #movement parameter gimmicks
        self.forward_speed = 400
        self.yaw_speed = 200

        #time loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("ROVMove node initialized")

    #Data callbacks
    def target_cb(self, msg: Point32):
        new_target = (msg.x, msg.y, msg.z)

        #If new target, then perceived position is updated
        if self.last_target is not None:
            self.assumed_x = self.last_target[0]
            self.assumed_y = self.last_target[1]
    
        self.current_target = new_target
        self.last_target = new_target

        self.get_logger().info(f"New target set: {self.current_target}, assumed pos ({self.assumed_x}, {self.assumed_y})")

        #Publishes depth setpoint
        depth_msg = Float64()
        depth_msg.data = msg.z
        self.pub_depth_setpoint.publish(depth_msg)

    def imu_cb(self, msg: Imu):
        # "convert quaternion → yaw"
        self.yaw = self.quat_to_yaw(msg.orientation)

    def depth_cb(self, msg: Float64):
        self.depth = msg.data

    # Main controls
    def control_loop(self):
        if self.current_target is None or self.yaw is None:
            return
    
        tx, ty, tz = self.current_target

        #computes relative vector from assumed pos
        dx = tx - self.assumed_x
        dy = ty - self.assumed_y

        #computes heading so it faces the next target
        desired_heading = math.degrees(math.atan2(dy, dx)) % 360

        #publish heading out to PID controller
        msg = Float64()
        msg.data = desired_heading
        self.pub_heading_setpoint.publish(msg)

        #move forward
        surge = self.forward_speed
        self.publish_manual(surge, 0.0, 0.0)

    #other helpful attachments
    def publish_manual(self, x, y, r):
        msg = ManualControl()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0 #leave depth to depth PID
        msg.r = float(r)
        self.pub_manual.publish(msg)

    def quat_to_yaw(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny = 2.0 * (w*z + x*y)
        cosy = 1.0 - 2.0 * (y*y + z*z)
        return math.degrees(math.atan2(siny, cosy)) % 360

    def angle_diff(self, target, current):
        return ((target - current + 540) % 360) - 180

def main(args=None):
    rclpy.init(args=args)
    node = ROVMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
