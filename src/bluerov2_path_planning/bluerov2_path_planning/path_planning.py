#Best case scenario: this computes shortest path & visit order for 5 pts using held karp algo :)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math
from functools import lru_cache
from geometry_msgs.msg import Polygon, Point32

class PathNode(Node):
    def __init__(self):
        super().__init__("path_planning_node")
        self.get_logger().info("Path publishing node initialized.")

        #Given points here
        start = (0.0, 0.0, 0.0)
        visit_points = [
            (1.0, 2.0, 3.0),
            (2.0, 3.0, 4.0),
            (3.0, 4.0, 5.0),
            (4.0, 5.0, 6.0),
            (5.0, 6.0, 7.0)
        ]

        #Publishes visit order of points
        self.publisher = self.create_publisher(
            Polygon,
            "/visit_order",
        )

        #Output from held_karp
        ordered_points = self.held_karp(start, visit_points)

        #Published output
        self.publish_order(ordered_points, visit_points)

    #Publish in optimized order
    def publish_order(self, ordered_points):
    msg = Polygon()

    for (x, y, z) in ordered_points:
        p = Point32()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        msg.points.append(p)

    self.publisher.publish(msg)
    self.get_logger().info(f"Published coordinate order with {len(msg.points)} points")

    #Held-Karp Algorithm
    def held_karp(self, start, visit_pointspoints):
        total_pts = [start] + visit_points
        n = len(total_pts)

        # Precompute distances
        D = [[math.dist(total_pts[i], total_pts[j]) for j in range(n)] for i in range(n)]

        @lru_cache(None)
        def dp(mask, last):
            if mask == (1 << last)):
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
            if cost < best_cost:
                best_cost = cost
                best_path = path

        #Reconvert into coordinates
        return [points[i-1] for i in best_path]

def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()