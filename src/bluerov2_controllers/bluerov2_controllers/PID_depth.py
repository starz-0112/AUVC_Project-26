# ros2 topic pub /target_depth std_msgs/msg/Float64 "{data: 5.0}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
from bluerov2_controllers import PIDController

class DepthPIDController(Node):
    def __init__(self):
        super().__init__('depth_lock')

        # --- PID setup with a dummy initial setpoint (0m), and anti-windup ---
        self.pid = PIDController.PIDController(
            kp=14.0, #70.0
            ki=0.55, #2.5
            kd=0.0, #5.0
            setpoint=0.0,
            dt=0.1,
        )

        # --- State ---
        self.current_depth = 0.0

        # --- Publishers & Subscribers ---
        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub = self.create_publisher(ManualControl, '/rov1/manual_control', 10)

        self.sub = self.create_subscription(
            Float64,
            '/depth',
            self.converting_depth,
            10
        )

        self.sub_setpoint = self.create_subscription(
            Float64,
            '/target_depth',
            self.setpoint_callback,
            10
        )

        self.timer = self.create_timer(self.pid.dt, self.control_loop)

        self.get_logger().info("Depth PID controller initialized")

    def setpoint_callback(self, msg: Float64):
        """Receive desired depth in meters for PID."""
        meters = msg.data
        self.pid.setpoint = meters
        self.get_logger().info(f"New setpoint: {meters:.2f} m")

    def converting_depth(self, msg: Float64):
        self.current_depth = msg.data
        self.get_logger().info(f"Current depth: {self.current_depth:.2f} meters")
        return self.current_depth

    def control_loop(self):
        thrust = self.pid.compute(self.current_depth)

        # Optionally clamp thrust here if needed
        # thrust = max(min(thrust, 1.0), -1.0)
        thrust = max(min(thrust, 600.0), -600.0)

        msg = Float64()
        msg.data = -thrust  # keep sign as-is for now, see note below
        self.pub_depth_thrust.publish(msg)

        self.get_logger().info(
            f"[PID] depth={self.current_depth:.3f} m  setpt={self.pid.setpoint:.3f} m  → thrust z={msg.data:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DepthPIDController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
