#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64MultiArray
import math

class FlashLights(Node):
    def __init__(self):
        super().__init__("flashy_light_node")

        # — Parameters you may tune —
        self.declare_parameter("flash_on_duration",    1.0)  # seconds ON
        self.declare_parameter("flash_off_duration",   1.0)  # seconds OFF
        self.declare_parameter("detection_timeout",    1.0)  # seconds before we drop flash
        self.declare_parameter("distance_threshold",   0.5)  # meters
        self.declare_parameter("max_flashes", 3)

        self.flash_on_duration   = self.get_parameter("flash_on_duration").value
        self.flash_off_duration  = self.get_parameter("flash_off_duration").value
        self.detection_timeout   = self.get_parameter("detection_timeout").value
        self.distance_threshold  = self.get_parameter("distance_threshold").value
        self.max_flashes = self.get_parameter("max_flashes").value

        # pub /sub
        self.command_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.create_subscription(
            Float64MultiArray, "/apriltag/detection",
            self.april_tag_callback, 10
        )
        self.create_subscription(
            Float64MultiArray, "/current_target",
            self.target_callback, 10
        )

        # state
        self.current_target_id = None
        self.should_flash = False
        self.light_on     = False
        self.last_toggle  = self.get_clock().now()
        self.last_det_t   = None

        self.timer = self.create_timer(0.1, self._on_timer)

    def target_callback(self, msg: Float64MultiArray):
        """Updates the active tag ID we should care about."""
        self.current_target_id = int(msg.data[0])
    
    def april_tag_callback(self, msg: Float64MultiArray):
        if self.current_target_id is None:
            return
        
        target_in_range = False

        for i in range(0, len(msg.data), 4):
            if i + 3 >= len(msg.data):
                break
                
            det_id = int(msg.data[i])
            tx = msg.data[i+1]
            ty = msg.data[i+2]
            tz = msg.data[i+3]

            # ONLY check the tag RouteManager is actively pursuing
            if det_id == self.current_target_id:
                dist = math.sqrt(tx**2 + ty**2 + tz**2)
                if dist < self.distance_threshold:
                    target_in_range = True
                    break

        # msg.data = [id, x, y, z, …]
        # dist = (msg.data[1]**2 + msg.data[2]**2 + msg.data[3]**2)**0.5
        # in_range = dist < self.distance_threshold

        if target_in_range:
            # start / refresh flash
            if not self.should_flash:
                self.get_logger().info("Tag in range → begin flashing")
                self.should_flash = True
                self.light_on = True
                self.last_toggle = self.get_clock().now()
                self._set_light_level(100)
            self.last_det_t  = self.get_clock().now()

        else:
            # immediate stop if tag is too far
            # if self.should_flash:
            #     self.get_logger().info("Tag out of range → stop flashing")
            # self.should_flash = False
            pass

    def _on_timer(self):
        t = self.get_clock().now()
        # expire stale detection
        if self.should_flash and self.last_det_t is not None:
            elapsed = (t - self.last_det_t).nanoseconds / 1e9

            if elapsed > self.detection_timeout:
                self.get_logger().info("Detection timeout → stop flashing")
                self.should_flash = False

        if self.should_flash:
            # handle on/off durations
            if self.light_on:
                elapsed = (t - self.last_toggle).nanoseconds / 1e9

                if elapsed >= self.flash_on_duration:
                    self.light_on    = False
                    self.last_toggle = t
                    self._set_light_level(0)
            else:
                elapsed = (t - self.last_toggle).nanoseconds / 1e9

                if elapsed >= self.flash_off_duration:
                    self.light_on    = True
                    self.last_toggle = t
                    self._set_light_level(100)
        else:
            # always force lights off
            if self.light_on:
                self.light_on = False
                self._set_light_level(0)

    def _set_light_level(self, level: int):
        """Map 0–100% → 1000–2000μs on channels 8 & 9."""
        pw = 1000 + level * 10
        cmd = OverrideRCIn()
        cmd.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        cmd.channels[8] = pw
        cmd.channels[9] = pw
        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FlashLights()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
