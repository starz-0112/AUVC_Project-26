#!/usr/bin/env python3

import os
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')

        self.bridge = CvBridge()

        # -------- CHANGE THESE PATHS --------
        self.images = [
            "/home/zoeg/AUVC_Project-26/src/TestImages/Tag1.jpeg",
            # "/home/zoeg/AUVC_Project-26/TestImages/Tag2.jpeg",
            # "/home/zoeg/AUVC_Project-26/TestImages/Tag3.jpeg",
            # "/home/zoeg/AUVC_Project-26/TestImages/Tag4.jpeg",
        ]
        # ------------------------------------

        self.index = 0

        self.publisher = self.create_publisher(
            Image,
            "camera",
            10
        )

        # Publish at 5 Hz
        self.timer = self.create_timer(
            0.2,
            self.publish_image
        )

        self.get_logger().info("Fake camera started.")

    def publish_image(self):

        filename = self.images[self.index]

        if not os.path.exists(filename):
            self.get_logger().error(f"Cannot find image: {filename}")
            return

        frame = cv2.imread(filename)

        if frame is None:
            self.get_logger().error(f"Could not read image: {filename}")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Published {os.path.basename(filename)}"
        )

        # Move to next image
        self.index = (self.index + 1) % len(self.images)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()