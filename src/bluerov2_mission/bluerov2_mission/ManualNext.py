#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ManualNext(Node):
    def __init__(self):
        super().__init__("manual_next")

        self.pub = self.create_publisher(Bool, '/manual_next', 10)
        self.get_logger().info("Press ENTER twice to skip to next waypoint")

        self.press_count = 0

        import threading
        threading.Thread(target=self.key_loop, daemon=True).start()

    def key_loop(self):
        while True:
            input()
            self.press_count += 1

            if self.press_count == 1:
                self.get_logger().info("Press ENTER again to confirm")
            elif self.press_count ==2:
                self.get_logger().info("ManualNext confirmed - skipping to next waypoint!")
                self.pub.publish(Bool(data=True))
                self.press_count = 0 #will reset count so next position can be skipped

def main(args=None):
    rclpy.init(args=args)
    node = ManualNext()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()