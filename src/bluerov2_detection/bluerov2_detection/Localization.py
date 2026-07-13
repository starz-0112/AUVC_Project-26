#!/usr/bin/env python3

#Current to-dos: will receive data from ReadAprilTags -> must use to verify estimated position 
# -> if ids are of a certain kind, recognize as waypoints and turn heading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64MultiArray, Int16
import math

class AprilTagLocalization(Node):
    def __init__(self):
        super().__init__('apriltag_localization_node')
        self.get_logger().info("AprilTag Localization node started!")

        # Known tag positions in "world frame"
        self.tag_locations = {
            10: (0.0, 0.0, 0.0),
            11: (1.0, 1.0, 1.0),
            12: (2.0, 2.0, 2.0),
            13: (2.0, 2.0, 0.25),
        }

        # Publishes position estimate
        self.position_pub = self.create_publisher(Point32, '/current_position', 10)
        # Subscribes to tag detection, IMU
        self.apriltag_sub = self.create_subscription(Float64MultiArray, 'apriltag/detection', self.detection_callback, 10) # Is going to output id, x, y, z in that order
        self.create_subscription(Int16, '/heading', self.heading_cb, 10)

        self.robot_heading = 0.0

    def heading_cb(self, msg: Int16):
        self.robot_heading_deg = float(msg.data)
    
    def detection_callback(self, msg: Float64MultiArray):
        data = msg.data

        if len(data) < 4:
            return # No tags in sight

        for i in range(0, len(data), 4):
            tag_id = int(data[i])
            tag_x = float(data[i+1])
            # tag_y = float(data[i+2])
            tag_z = float(data[i+3])

            if tag_id not in self.tag_locations:
                continue # Skips unknown tags

            tag_real_x, tag_real_y, tag_real_z = self.tag_locations[tag_id]

            #Do localization stuff here
            tag_bearing = math.atan2(tag_x, tag_z) # Radians
            robot_heading = math.radians(90.0 - self.robot_heading_deg)
            true_tag_bearing = robot_heading + tag_bearing
            r = math.hypot(tag_x, tag_z)

            #Estmated position
            robot_x = tag_real_x - r * math.cos(true_tag_bearing)
            robot_y = tag_real_y - r * math.sin(true_tag_bearing)

            #Publishes estimated position
            msg_out = Point32()
            msg_out.x = robot_x
            msg_out.y = robot_y

            self.position_pub.publish(msg_out)
            self.get_logger().info(f"Sensed Tag #{tag_id}, and estimated position at ({robot_x: .2f}, {robot_y: .2f})")

            #Suggested logger for debugging:
            self.get_logger().info(
                f"""
                Tag: ({tag_real_x}, {tag_real_y})
                Measured: x={tag_x:.2f}, z={tag_z:.2f}
                Heading={self.robot_heading_deg:.1f}°
                Relative bearing={math.degrees(tag_bearing):.1f}°
                World bearing={math.degrees(true_tag_bearing):.1f}°
                Estimated robot=({robot_x:.2f}, {robot_y:.2f})
                """
            )


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()