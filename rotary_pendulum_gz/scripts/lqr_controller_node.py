#!/usr/bin/env python3

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Float64


class LqrControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("lqr_controller_node")

        self.declare_parameter("k", [10.0, 2.0, 120.0, 12.0])
        self.declare_parameter("voltage_limit", 24.0)
        self.declare_parameter("control_rate_hz", 100.0)

        self.k = [float(v) for v in self.get_parameter("k").value]
        if len(self.k) != 4:
            raise ValueError("Parameter 'k' must have exactly 4 gains: [k_arm, k_arm_vel, k_pole, k_pole_vel]")

        self.voltage_limit = float(self.get_parameter("voltage_limit").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.arm_angle = None
        self.arm_vel = None
        self.pole_angle = None
        self.pole_vel = None

        self.voltage_cmd_pub = self.create_publisher(Float64, "/rotary_pendulum/motor_voltage_cmd", 10)

        self.create_subscription(Float64, "/rotary_pendulum/arm_angle", self.arm_angle_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/arm_angular_velocity", self.arm_vel_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/pole_angle", self.pole_angle_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/pole_angular_velocity", self.pole_vel_cb, 20)

        wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(1.0 / self.control_rate_hz, self.control_loop, clock=wall_clock)
        self.create_timer(1.0, self.status_log_loop, clock=wall_clock)
        self.get_logger().info(
            f"LQR controller started. K={self.k}, voltage_limit={self.voltage_limit}, rate={self.control_rate_hz}Hz"
        )

    def arm_angle_cb(self, msg: Float64) -> None:
        self.arm_angle = float(msg.data)

    def arm_vel_cb(self, msg: Float64) -> None:
        self.arm_vel = float(msg.data)

    def pole_angle_cb(self, msg: Float64) -> None:
        self.pole_angle = float(msg.data)

    def pole_vel_cb(self, msg: Float64) -> None:
        self.pole_vel = float(msg.data)

    def control_loop(self) -> None:
        if None in (self.arm_angle, self.arm_vel, self.pole_angle, self.pole_vel):
            return

        x = [self.arm_angle, self.arm_vel, self.pole_angle, self.pole_vel]
        u = -sum(k_i * x_i for k_i, x_i in zip(self.k, x))

        if u > self.voltage_limit:
            u = self.voltage_limit
        elif u < -self.voltage_limit:
            u = -self.voltage_limit

        msg = Float64()
        msg.data = u
        self.voltage_cmd_pub.publish(msg)

    def status_log_loop(self) -> None:
        missing = []
        if self.arm_angle is None:
            missing.append("arm_angle")
        if self.arm_vel is None:
            missing.append("arm_angular_velocity")
        if self.pole_angle is None:
            missing.append("pole_angle")
        if self.pole_vel is None:
            missing.append("pole_angular_velocity")

        if missing:
            self.get_logger().warn(f"Waiting for state topics: {', '.join(missing)}")


def main() -> None:
    rclpy.init()
    node = LqrControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
