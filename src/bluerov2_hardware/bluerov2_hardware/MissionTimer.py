#!/usr/bin/env python3
#Subscribes to topic /mission_status that will publish True when mission is started and False when ended
#Once started, will begin counting time until next False value. Outputs this message so if sub to topic, can find info


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

class MissionTimer(Node):
    def __init__(self):
        super().__init__("mission_timer_node")
        self.get_logger().info("Mission timer initialized.")

        # high-rate timer
        self.timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info("Timer ready")

        #MISSION TIMERS HERE
        # Mission timing
        self.mission_started = False
        self.mission_start_time = None

        # Subscribe to mission status
        self.status_sub = self.create_subscription(
            Bool,
            '/mission_status',
            self.status_callback,
            10
        )

        # Publishers for mission time (yay only need to echo one topic)
        self.time_pub = self.create_publisher(Float64, '/mission_time', 10)
    
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9
    
    def status_callback(self, msg: Bool):
        # Start the timer if it is not already running
        if msg.data:
            if not self.mission_started:
                self.mission_started = True
                self.mission_start_time = self.now()
                self.get_logger().info("Mission is a go! Timer has started.")

        # Once false bool is received, shut the timer off and pub final time
        else:
            if self.mission_started:
                total_time = self.now() - self.mission_start_time
                self.get_logger().info("Mission is inactive.")
                self.get_logger().info(f"Total elapsed mission time: {total_time:.3f} seconds.")

                time_msg = Float64(data=float(total_time))
                self.time_pub.publish(time_msg)

                # Reset state for the next run
                self.mission_started = False
                self.mission_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = MissionTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()