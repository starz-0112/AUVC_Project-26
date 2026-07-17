#Best case scenario: this computes shortest path & visit order for 5 pts using held karp algo :)
#Note to implement later: each waypoint must also store the ID of the tag at that waypoint!

import rclpy
from rclpy.node import Node
import math
from functools import lru_cache
from std_msgs.msg import Float64MultiArray

SPEED_MPS = 8.35 #In m/s
MAX_LEG_TIME = 60.0 # Available time, in sec, for the robot to drain battery

class PathNode(Node):
    def __init__(self):
        super().__init__("path_planning_node")
        self.get_logger().info("Path publishing node initialized.")

        #Given points here
        dock = (0, 0.0, 0.0, 0.0)
        visit_points = [
            (6, 4.62, 0.0, 0.0),
            (7, 0.0, -1.0, 0.0),
            (8, 4.62, -2.0, 0.0),
            (9, 0.0, -3.0, 0.0),
            (10, 4.62, -4.0, 0.0)
        ]

        #Publishes visit order of points
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/visit_order',
            10
        )
        
        hk_route = self.held_karp(dock, visit_points)
        self.get_logger().info(f"Held-Karp: {hk_route}")

        battery_route = self.apply_time_constraints(dock, hk_route)
        self.get_logger().info(f"Battery-aware route: {battery_route}")

        #Published output
        self.route = battery_route

        self.timer = self.create_timer(1.0, self.publish_order_timer)

    def publish_order_timer(self):
        self.publish_order(self.route)
    
    #Publish in optimized order
    def publish_order(self, ordered_points):
        msg = Float64MultiArray()

        for (tag_id, x, y, z) in ordered_points:
            msg.data.extend([
                float(tag_id),
                float(x),
                float(y),
                float(z)
            ])

        self.publisher.publish(msg)
        self.get_logger().info(f"Published coordinate order with {len(ordered_points)} points")

    #Held-Karp Algorithm
    def held_karp(self, dock, visit_points):
        all_points = [dock] + visit_points
        n = len(visit_points)

        # Precompute distances - how does this account for the id of the tag at the front?
        D = [[math.dist(all_points[i][1:], all_points[j][1:]) for j in range(n + 1)] for i in range(n + 1)]

        @lru_cache(None)
        def dp(mask, last):
            if mask == (1 << last):
                return D[0][last + 1], [last]
            
            best_cost = float('inf')
            best_path = None
            prev_mask = mask ^ (1 << last)

            for k in range(n):
                if prev_mask & (1 << k):
                    cost, path = dp(prev_mask, k)
                    new_cost = cost + D[k + 1][last + 1]
                    if new_cost < best_cost:
                        best_cost = new_cost
                        best_path = path + [last]
                        
            return best_cost, best_path

        full_mask = (1 << n) - 1
        best_cost = float('inf')
        best_path = None

        for last in range(n):
            cost, path = dp(full_mask, last)
            cost += D[last + 1][0] #returns to start (dock) position
            if cost < best_cost:
                best_cost = cost
                best_path = path

        #Reconvert into coordinates
        if best_path is None:
            raise RuntimeError("Held-Karp failed to find a valid path.")

        route = [dock]

        for idx in best_path:
            route.append(visit_points[idx])

        route.append(dock)

        return route

    def apply_time_constraints(self, dock, route):
     # This one only slots in stops regularly - it doesn't account global awareness stuffs

        if not route:
            return []

        battery_route = [route[0]]
        leg_time = 0.0

        for i in range(len(route) - 1):
            p_curr = route[i]
            p_next = route[i+1]

            # Time from current point to next point
            seg_time = math.dist(p_curr[1:], p_next[1:]) / SPEED_MPS

            # Time needed to return home FROM the next point
            return_time = math.dist(p_next[1:], dock[1:]) / SPEED_MPS

            # Can we safely reach the next point AND still make it home?
            if leg_time + seg_time + return_time > MAX_LEG_TIME:
                if battery_route and battery_route[-1] != dock:
                    battery_route.append(dock)
                    self.get_logger().info("Battery limit reached, heading to dock")
                
                leg_time = 0.0
                seg_time = math.dist(dock[1:], p_next[1:]) / SPEED_MPS

            battery_route.append(p_next)
            leg_time += seg_time

        if battery_route[-1] != dock:
            battery_route.append(dock)
        
        clean_route = []
        for point in battery_route:
            if not clean_route or point != clean_route[-1]:
                clean_route.append(point)

        return clean_route
    
def main(args=None):
    rclpy.init(args=args)
    node = PathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()