#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl # movement

class SLAM(Node):
    def __init__(self):
        super().__init__('SLAM_node')
        self.get_logger().info("SLAM node started!")

        # To be tweaked
        self.surge_command = 0.0
        self.sway_command = 0.0

        # TUNE LATER! THIS SHOULD PROBABLY MATCH MAINMOVEMENT INFO
        self.max_manual = 400.0
        self.max_speed = 0.22   # tune experimentally

        # Estimated positionings - NOT SUPER ACCURATE
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.robot_depth = 0.0

        self.heading_deg = 0.0

        # Time since last update for dead-reckoning (estimated position)
        self.last_time = self.get_clock().now()

        # Will eventually store tag IDs and their guesstimate positions
        self.landmarks = {}

        # Tags on the route/future targets that most definitely are not moving
        self.fixed_tags = {

            0: (0.0, 3.5),
            1: (3.5, 0.0),
            2: (-3.5, 0.0),
            3: (0.0, -3.5),
            4: (0.0, -3.5),
            5: (0.0, 0.0)
        }

        # Pubs
        self.position_pub = self.create_publisher(Point32, "/current_position", 10)

        #Subs
        # Will give an array with [id, dx, dy, dz, stuffs]
        self.create_subscription(ManualControl, '/manual_control', self.manual_control_callback, 10)
        self.create_subscription(ManualControl, '/rov1/manual_control', self.manual_control_callback, 10)

        self.create_subscription(Float64MultiArray, "apriltag/detection", self.apriltag_callback, 10)
            
        self.create_subscription(Int16, "/heading", self.heading_callback, 10)
        self.create_subscription(Int16, "/rov1/heading", self.heading_callback, 10)

        self.create_subscription(Float64, "/depth", self.depth_callback, 10)
        # self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.timer = self.create_timer(0.05, self.dead_reckoning) # 20 Hz

    def manual_control_callback(self, msg):
        self.surge_command = msg.x
        self.sway_command = msg.y
    
    def heading_callback(self, msg: Int16):
        raw = float(msg.data)
        if self.heading_offset is None:
            if raw <= 90.0:
                self.heading_offset = 90.0 - raw
            else:
                self.heading_offset = raw - 90.0
            self.get_logger().info(f"SLAM heading offset initialized: {self.heading_offset:.1f}°")

        if raw <= 90.0:
            self.heading_deg = (raw + self.heading_offset) % 360
        else:
            self.heading_deg = (raw - self.heading_offset) % 360
    
    def depth_callback(self, msg: Float64):
        self.robot_depth = float(msg.data)
    
    # def imu_callback(self, msg: Imu):
        # Keep in mind that because we think IMU is untrustworthy, we aren't integrating it - we're using direct velocity instead
        # self.ax = msg.linear_acceleration.x
        # self.ay = msg.linear_acceleration.y
        # self.az = msg.linear_acceleration.z

        # self.wx = msg.angular_velocity.x
        # self.wy = msg.angular_velocity.y
        # self.wz = msg.angular_velocity.z

    def dead_reckoning(self):
        # Position estimation method - this is the section I will probably have to adjust the most
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0:
            return # Aka you can't update instantly
        heading = math.radians(self.heading_deg)
        
        surge_speed = (self.surge_command / self.max_manual) * self.max_speed
        sway_speed  = (self.sway_command  / self.max_manual) * self.max_speed

        dx = (surge_speed * math.sin(heading) + sway_speed  * math.cos(heading)) * dt
        dy = (surge_speed * math.cos(heading) - sway_speed  * math.sin(heading)) * dt

        self.robot_x += dx
        self.robot_y += dy

        self.publish_position()

    def camera_to_world_offset(self, camera_x, camera_y, camera_z):
        # This does NOT calculate tag position, but a set of (dx, dy, and dz) that can eventually be used for 3 different purposes:
            # To "create a new landmark, compare against an existing landmark, or correct robot position"
        horizontal_distance = math.sqrt(camera_x**2 + camera_z**2)

        # Tag bearing
        relative_bearing = math.atan2(camera_x, camera_z)
        world_bearing = math.radians(self.heading_deg) + relative_bearing

        # Rotate into world frame
        dx = horizontal_distance * math.sin(world_bearing)
        dy = horizontal_distance * math.cos(world_bearing)

        # Be careful with sign on this one:
        # If positive camera_y means the tag is LOWER than the camera, then tag_depth = robot_depth + camera_y. Otherwise, need to have dz = -camera_y
        dz = camera_y

        return dx, dy, dz
    
    def apriltag_callback(self, msg: Float64MultiArray):
        # The detector publishes data in groups of four values:
        # [id, x, y, z,
        # id, x, y, z,
        # ...]

        # x = left/right from camera
        # y = up/down from camera
        # z = forward distance

        data = msg.data

        if len(data) == 0:
            return # Returns empty if no tags detected
        
        for i in range(0, len(data), 4):
            if i + 3 >= len(data):
                break
            
            tag_id = int(data[i])

            camera_x = float(data[i + 1])
            camera_y = float(data[i + 2])
            camera_z = float(data[i + 3])

            if tag_id in self.fixed_tags:

                self.correct_from_fixed_tag(
                    tag_id,
                    camera_x,
                    camera_y,
                    camera_z
                )

                continue

            if tag_id in self.landmarks:

                self.update_landmark(
                    tag_id,
                    camera_x,
                    camera_y,
                    camera_z
                )

            else:
                self.create_landmark(
                    tag_id,
                    camera_x,
                    camera_y,
                    camera_z
                )

    def create_landmark(self, tag_id, camera_x, camera_y, camera_z):
        dx, dy, dz = self.camera_to_world_offset(camera_x, camera_y, camera_z)

        # Now turn to world form
        landmark_x = self.robot_x + dx
        landmark_y = self.robot_y + dy
        landmark_z = self.robot_depth + dz

        # Store the landmark in the thing w/ estimated position
        self.landmarks[tag_id] = {
        "x": landmark_x,
        "y": landmark_y,
        "z": landmark_z,
        "times_seen": 1,
        "last_distance": math.sqrt(camera_x**2 + camera_y**2 + camera_z**2), # This and below have practically no use other than "is this broken"
        "last_bearing": math.degrees(math.atan2(camera_x, camera_z))
        }

        self.get_logger().info(f"Created landmark {tag_id} at {landmark_x:.2f}, {landmark_y:.2f}, {landmark_z:.2f})")

    def update_landmark(self, tag_id, camera_x, camera_y, camera_z):
        dx, dy, dz = self.camera_to_world_offset(camera_x, camera_y, camera_z)

        measured_x = self.robot_x + dx
        measured_y = self.robot_y + dy
        measured_z = self.robot_depth + dz

        landmark = self.landmarks[tag_id]
        n = landmark["times_seen"]

        landmark["x"] = (landmark["x"] * n + measured_x) / (n + 1) # Serves as an averaging method
        landmark["y"] = (landmark["y"] * n + measured_y) / (n + 1)
        landmark["z"] = (landmark["z"] * n + measured_z) / (n + 1)

        landmark["times_seen"] += 1

        landmark["last_distance"] = math.sqrt(camera_x**2 + camera_y**2 + camera_z**2)
        landmark["last_bearing"] = math.degrees(math.atan2(camera_x, camera_z))

        self.get_logger().debug(f"Updated landmark {tag_id} (seen {landmark['times_seen']} times)")

    def correct_from_fixed_tag(self, tag_id, camera_x, camera_y, camera_z):
        tag_world_x, tag_world_y = self.fixed_tags[tag_id]

        dx, dy, dz = self.camera_to_world_offset(camera_x, camera_y, camera_z)

        measured_robot_x = tag_world_x - dx
        measured_robot_y = tag_world_y - dy
        # measured_robot_z = tag_world_z - dz

        # Gotta research more about this correction thing here:
        correction_gain = 0.3

        self.robot_x = ((1.0 - correction_gain) * self.robot_x + correction_gain * measured_robot_x)
        self.robot_y = ((1.0 - correction_gain) * self.robot_y + correction_gain * measured_robot_y)

        self.get_logger().info(f"Localization correction from tag {tag_id}")

    def publish_position(self):
        # Literally just to publish its current position; arguably the most important method for debugging purposes
        position = Point32()

        position.x = self.robot_x
        position.y = self.robot_y
        position.z = self.robot_depth

        self.position_pub.publish(position)

def main(args=None):
    rclpy.init(args=args)
    node = SLAM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()