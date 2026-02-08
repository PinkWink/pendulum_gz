#!/usr/bin/env python3

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class LqrControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("lqr_controller_node")

        self.declare_parameter("k", [10.0, 2.0, 120.0, 12.0])
        self.declare_parameter("voltage_limit", 24.0)
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("publish_zero_if_state_missing", True)
        self.declare_parameter("arm_joint_name", "arm_joint")
        self.declare_parameter("pole_joint_name", "pole_joint")
        self.declare_parameter("pole_initial_offset_deg", 3.0)

        self.k = [float(v) for v in self.get_parameter("k").value]
        if len(self.k) != 4:
            raise ValueError("Parameter 'k' must have exactly 4 gains: [k_arm, k_arm_vel, k_pole, k_pole_vel]")

        self.voltage_limit = float(self.get_parameter("voltage_limit").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.publish_zero_if_state_missing = bool(self.get_parameter("publish_zero_if_state_missing").value)
        self.arm_joint_name = str(self.get_parameter("arm_joint_name").value)
        self.pole_joint_name = str(self.get_parameter("pole_joint_name").value)
        self.pole_offset = float(self.get_parameter("pole_initial_offset_deg").value) * 3.141592653589793 / 180.0

        self.arm_angle = None
        self.arm_vel = None
        self.pole_angle = None
        self.pole_vel = None
        self.last_u = 0.0
        self._state_ready_logged = False

        self.voltage_cmd_pub = self.create_publisher(Float64, "/rotary_pendulum/motor_voltage_cmd", 10)

        self.create_subscription(Float64, "/rotary_pendulum/arm_angle", self.arm_angle_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/arm_angular_velocity", self.arm_vel_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/pole_angle", self.pole_angle_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/pole_angular_velocity", self.pole_vel_cb, 20)
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 20)

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

    def _find_joint_index(self, names: list[str], target: str):
        if target in names:
            return names.index(target)
        for i, name in enumerate(names):
            if name.endswith("/" + target) or name.endswith("::" + target):
                return i
        return None

    def joint_states_cb(self, msg: JointState) -> None:
        ai = self._find_joint_index(msg.name, self.arm_joint_name)
        if ai is not None:
            if ai < len(msg.position):
                self.arm_angle = float(msg.position[ai])
            if ai < len(msg.velocity):
                self.arm_vel = float(msg.velocity[ai])

        pi = self._find_joint_index(msg.name, self.pole_joint_name)
        if pi is not None:
            if pi < len(msg.position):
                # Keep the same state convention used by pendulum_interface_node.
                self.pole_angle = float(msg.position[pi]) + self.pole_offset
            if pi < len(msg.velocity):
                self.pole_vel = float(msg.velocity[pi])

    def control_loop(self) -> None:
        if None in (self.arm_angle, self.arm_vel, self.pole_angle, self.pole_vel):
            if self.publish_zero_if_state_missing:
                msg = Float64()
                msg.data = 0.0
                self.voltage_cmd_pub.publish(msg)
                self.last_u = 0.0
            return

        x = [self.arm_angle, self.arm_vel, self.pole_angle, self.pole_vel]
        u = sum(k_i * x_i for k_i, x_i in zip(self.k, x))

        if u > self.voltage_limit:
            u = self.voltage_limit
        elif u < -self.voltage_limit:
            u = -self.voltage_limit

        msg = Float64()
        msg.data = u
        self.voltage_cmd_pub.publish(msg)
        self.last_u = u

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
            self._state_ready_logged = False
            self.get_logger().warn(f"Waiting for state topics: {', '.join(missing)}")
            return

        if not self._state_ready_logged:
            self.get_logger().info(
                "State input ready. "
                f"x=[{self.arm_angle:.3f}, {self.arm_vel:.3f}, {self.pole_angle:.3f}, {self.pole_vel:.3f}], "
                f"u={self.last_u:.3f} V"
            )
            self._state_ready_logged = True


def main() -> None:
    rclpy.init()
    node = LqrControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
