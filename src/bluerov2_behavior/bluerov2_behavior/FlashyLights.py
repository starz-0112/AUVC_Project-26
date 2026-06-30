#NOT JUST LIGHTS!!! - stores route, waypoint progression, AprilTag triggers, mission timer, final kill signal

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Bool

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
        self.command_pub = self.create_publisher(OverrideRCIn, "override_rc", 10)
        self.create_subscription(
            Float64MultiArray, "apriltag/detection",
            self.april_tag_callback, 10
        )

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

        # state
        self.should_flash = False
        self.light_on     = False
        self.last_toggle  = self.now()
        self.last_det_t   = None

        # Store the route from Held–Karp
        self.route = []
        self.flash_count = 0

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


    def route_callback(self, msg: Polygon):
        # Convert Polygon → list of (x, y, z)
        self.route = [(p.x, p.y, p.z) for p in msg.points]
        self.get_logger().info(f"Received route with {len(self.route)} points")

        # Immediately publish the first target
        if self.route:
            self.publish_current_target()
        
        # Mission Timer
        if not self.mission_started:
            self.mission_started = True
            self.mission_start_time = self.now()
            self.get_logger().info("Mission timer started")


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
            else:
                # Mission complete
                total = self.now() - self.mission_start_time
                self.get_logger().info(f"MISSION COMPLETE — total time: {total:.2f} seconds")

                # Kill signal
                end_msg = Bool()
                end_msg.data = False
                self.end_pub.publish(end_msg)

                # Publish mission time
                tmsg = Float64()
                tmsg.data = float(total)
                self.time_pub.publish(tmsg)

        else:
            # immediate stop if tag is too far
            if self.should_flash:
                self.get_logger().info("Tag out of range → stop flashing")
            self.should_flash = False

    def _on_timer(self):
        t = self.now()
        # expire stale detection
        if self.should_flash and self.last_det_t is not None:
            if (t - self.last_det_t) > self.detection_timeout:
                self.get_logger().info("Detection timeout → stop flashing")
                self.should_flash = False

        if self.should_flash:
            # handle on/off durations
            if self.light_on:
                if (t - self.last_toggle) >= self.flash_on_duration:
                    self.light_on    = False
                    self.last_toggle = t
                    self._set_light_level(0)
            else:
                if (t - self.last_toggle) >= self.flash_off_duration:
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