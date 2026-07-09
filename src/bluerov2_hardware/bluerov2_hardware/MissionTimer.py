#!/usr/bin/env python3
#Subscribes to topic /mission_status that will publish True when mission is started and False when ended
#Once started, will begin counting time until next False value. Outputs this message so if sub to topic, can find info


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

class FlashLights(Node):
    def __init__(self):
        super().__init__("mission_timer_node")

        # high-rate timer
        self.timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info("FlashLights ready")

        #MISSION TIMERS HERE
        # Mission timing
        self.mission_started = False
        self.mission_start_time = None

        # Publishers for kill + mission time
        self.end_pub = self.create_publisher(Bool, "/mission_end", 10)
        self.time_pub = self.create_publisher(Float64, "/mission_time", 10)
    
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9
    
    def start_mission(self):
        # Mission Timer
        if not self.mission_started:
            self.mission_started = True
            self.mission_start_time = self.now()
            self.get_logger().info("Mission timer started")

    def current_target(self):
        # If route empty, then mission complete
        if not self.route:
            self.get_logger().info("Mission complete — returned to start")
            
            if self.mission_started:
                total = self.now() - self.mission_start_time
                self.get_logger().info(f"Total mission time {total: .3f} seconds")

                # Publishes mission time
                tmsg = Float64()
                tmsg.data = float(total)
                self.time_pub.publish(tmsg)
            return

def main(args=None):
    rclpy.init(args=args)
    node = FlashLights()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()