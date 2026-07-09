#Best case scenario: this computes shortest path & visit order for 5 pts using held karp algo :)
#Note to implement later: each waypoint must also store the ID of the tag at that waypoint!

import rclpy
from rclpy.node import Node
import math
from functools import lru_cache
from geometry_msgs.msg import Polygon, Point32

SPEED_MPS = 0.2 #In m/s
MAX_LEG_TIME = 20.0 # Available time, in sec, for the robot to drain battery

class PathNode(Node):
    def __init__(self):
        super().__init__("path_planning_node")
        self.get_logger().info("Path publishing node initialized.")

        #Given points here
        dock = (0, 0.0, 0.0, 0.0)
        visit_points = [
            (1, 1.0, 2.0, 3.0),
            (2, 2.0, 3.0, 4.0),
            (3, 3.0, 4.0, 5.0),
            (4, 4.0, 5.0, 6.0),
            (5, 5.0, 6.0, 7.0)
        ]

        #Publishes visit order of points
        self.publisher = self.create_publisher(
            Polygon,
            "/visit_order",
            10
        )
        
        hk_route = self.held_karp(dock, visit_points)
        self.get_logger().info(f"Held-Karp: {hk_route}")

        battery_route = self.apply_time_constraints(dock, hk_route)
        self.get_logger().info(f"Battery-aware route: {battery_route}")

        #Published output
        self.publish_order(battery_route)

    #Publish in optimized order
    def publish_order(self, ordered_points):
        msg = Polygon()

        for (a, x, y, z) in ordered_points:
            p = Point32()
            p.id = a
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            msg.points.append(p)

        self.publisher.publish(msg)
        self.get_logger().info(f"Published coordinate order with {len(msg.points)} points")

    #Held-Karp Algorithm
    def held_karp(self, dock, visit_points):
        total_pts = [dock] + visit_points
        n = len(total_pts)

        # Precompute distances - how does this account for the id of the tag at the front?
        D = [[math.dist(total_pts[i], total_pts[j]) for j in range(n)] for i in range(n)]

        @lru_cache(None)
        def dp(mask, last):
            if mask == (1 << last):
                return D[0][last], [last]
            
            best_cost = float('inf')
            best_path = None
            prev_mask = mask ^ (1 << last)

            for k in range(1, n):
                if prev_mask & (1 << k):
                    cost, path = dp(prev_mask, k)
                    new_cost = cost + D[k][last]
                    if new_cost < best_cost:
                        best_cost = new_cost
                        best_path = path + [last]
                        
            return best_cost, best_path

        full_mask = (1 << n) - 1
        best_cost = float('inf')
        best_path = None

        for last in range(1, n):
            cost, path = dp(full_mask, last)
            cost += D[last][0] #returns to start (dock) position
            if cost < best_cost:
                best_cost = cost
                best_path = path + [0] #append start (dock)

        #Reconvert into coordinates
        return [total_pts[i] for i in best_path]

    def apply_time_constraints(self, dock, route):
     # This one only slots in stops regularly - it doesn't account global awareness stuffs

        if not route:
            return []

        battery_route = []
        leg_time = 0.0

        for i in range(len(route) - 1):
            p_curr = route[i]
            p_next = route[i+1]

            dist = math.dist(p_curr, p_next)
            seg_time = dist / SPEED_MPS

            if leg_time + seg_time > MAX_LEG_TIME:
                if battery_route and battery_route[-1] != dock:
                    battery_route.append(dock)
                    self.get_logger().info("Battery limit reached, heading to dock")
            
                leg_time = math.dist(dock[1:3], p_next[1:3]) / SPEED_MPS
                battery_route.append(dock)
                battery_route.append(p_next)
            else:
                if not battery_route:
                    battery_route.append(p_curr)
                battery_route.append(p_next)
                leg_time += seg_time
        if battery_route[-1] != dock:
            battery_route.append(dock)

        return battery_route

def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()