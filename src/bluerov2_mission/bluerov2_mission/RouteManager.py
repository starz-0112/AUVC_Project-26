#!/usr/bin/env python3
#RouteManager only continuously publishes the next waypoint
#Receives route from path_planner

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
import math

class RouteManager(Node):
    def __init__(self):
        super().__init__("route_manager_node")
        self.get_logger().info("RouteManager node ready!")
        self.route_received = False

        self.max_tag_distance = 0.2
        self.dock_id = 0

        # Subscribe to the optimized route
        self.create_subscription(
            Float64MultiArray,
            '/visit_order',
            self.route_callback, 10
        )

        # Subscribes to skip method
        self.create_subscription(
            Bool,
            '/manual_next',
            self.manual_next_callback, 10
        )

        self.create_subscription(
            Float64MultiArray,
            '/apriltag/detection',
            self.april_tag_callback, 10
        )

        self.target_pub = self.create_publisher(Float64MultiArray, '/current_target', 10)
        self.status_pub = self.create_publisher(Bool, '/mission_status', 10)

        self.timer = self.create_timer(0.2, self.publish_current_target)

        # Store the route from Held–Karp
        self.route = []


    def route_callback(self, msg: Float64MultiArray):
        if self.route_received:
            return
        
        self.route_received = True
        
        # Convert Polygon → list of (id, x, y, z)
        full_route = []

        # msg.data = [id, x, y, z, id, x, y, z, ...]
        for i in range(0, len(msg.data), 4):
            if i + 3 >= len(msg.data):
                break

            full_route.append((
                int(msg.data[i]),
                msg.data[i + 1],
                msg.data[i + 2],
                msg.data[i + 3]
            ))
        self.get_logger().info(f"Received route with {len(full_route)} points")
        
        if not full_route:
            return
        
        #Skips first target point if it's marked as the dock
        if full_route[0][0] == self.dock_id:
            self.get_logger().info("Skipping initial dock target.")
            self.route = full_route[1:]
        else:
            self.route = full_route

        # Immediately publish the first target
        if self.route:
            self.publish_status(True)
            self.publish_current_target()

    def publish_status(self, mission_status: Bool):
        status_msg = Bool()
        status_msg.data = mission_status
        self.status_pub.publish(status_msg)

    def publish_current_target(self):
        if not self.route_received:
            return

        
        # If route empty, then mission complete
        if not self.route:
            self.get_logger().info("Mission complete — returned to start")
            self.publish_status(False)
            return
        
        target_id, x, y, z = self.route[0]

        msg = Float64MultiArray()
        msg.data = [
            float(target_id),
            float(x),
            float(y),
            float(z)
        ]
        self.target_pub.publish(msg)
        self.get_logger().info(f"Current target ID: {target_id} at ({x}, {y}, {z})")

    def manual_next_callback(self, msg):
        self.get_logger().info(f"Received manual_next: {msg.data}")
        if msg.data:
            if self.route:
                popped = self.route.pop(0)
                self.get_logger().info(f"MANUAL ADVANCE → Skipped {popped}")
                self.publish_current_target()
            else:
                self.get_logger().warn("No more points to advance to")

    def april_tag_callback(self, msg: Float64MultiArray):
        if not self.route:
            return
        
        target_id, target_x, target_y, target_z = self.route[0]
        # msg.data = [id, x, y, z, …]
        for i in range(0, len(msg.data), 4):
            if i + 3 >= len(msg.data):
                break
                
            det_id = int(msg.data[i])
            tx = msg.data[i+1]
            ty = msg.data[i+2]
            tz = msg.data[i+3]
            
            # Match current target ID
            if det_id == target_id:
                dist = math.sqrt(tx**2 + ty**2 + tz**2)
                
                # Check range constraint
                if dist < self.max_tag_distance:
                    self.get_logger().info(f"Reached tag #{target_id}, moving to next point!")
                    self.route.pop(0)
                    self.publish_current_target()
                    break

def main(args=None):
    rclpy.init(args=args)
    node = RouteManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()