#!/usr/bin/env python3
#RouteManager only continuously publishes the next waypoint
#Receives route from path_planner

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Bool, Float64, Float64MultiArray

class RouteManager(Node):
    def __init__(self):
        super().__init__("route_manager_node")

        # Subscribe to the optimized route
        self.create_subscription(
            Polygon,
            "/visit_order",
            self.route_callback, 10
        )

        # Subscribes to skip method
        self.create_subscription(
            Bool,
            "/manual_next",
            self.manual_next_callback, 10
        )

        self.target_pub = self.create_publisher(Point32, "/current_target", 10)

        # Store the route from Held–Karp
        self.route = []


    def route_callback(self, msg: Polygon):
        # Convert Polygon → list of (x, y, z)
        self.route = [(p.x, p.y, p.z) for p in msg.points]
        self.get_logger().info(f"Received route with {len(self.route)} points")

        # Immediately publish the first target
        if self.route:
            self.publish_current_target()


    def publish_current_target(self):
        # If route empty, then mission complete
        if not self.route:
            self.get_logger().info("Mission complete — returned to start")
            
            if self.mission_started:
                total = self.now() - self.mission_start_time
                self.get_logger().info(f"Total mission time {total: .3f} seconds")

                # Kill signal
                end_msg = Bool()
                end_msg.data = False
                self.end_pub.publish(end_msg)

                # Publishes mission time
                tmsg = Float64()
                tmsg.data = float(total)
                self.time_pub.publish(tmsg)
            
            return

        # Else, publish next target
        x, y, z = self.route[0]
        msg = Point32(x=float(x), y=float(y), z=float(z))
        self.target_pub.publish(msg)
        self.get_logger().info(f"Published current target: {msg}")

    def manual_next_callback(self, msg):
        if msg.data:
            if self.route:
                popped = self.route.pop(0)
                self.get_logger().info(f"MANUAL ADVANCE → Skipped {popped}")
                self.publish_current_target()
            else:
                self.get_logger().warn("No more points to advance to")

    
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def april_tag_callback(self, msg: Float64MultiArray):
        # msg.data = [id, x, y, z, …]
        dist = (msg.data[1]**2 + msg.data[2]**2 + msg.data[3]**2)**0.5
        in_range = dist < self.distance_threshold

        if in_range:
            #Biggest problem is that this needs to match the ID of the waypoint, which will be structured as [id, x, y, z]
            # start / refresh flash
            if not self.should_flash:
                self.get_logger().info("Tag in range → begin flashing")
            self.should_flash = True
            self.last_det_t  = self.now()

            # Pop the first coordinate and publish the next one
            if self.route:
                popped = self.route.pop(0)
                self.get_logger().info(f"Reached {popped}, moving to next point")
                self.publish_current_target()

def main(args=None):
    rclpy.init(args=args)
    node = RouteManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()