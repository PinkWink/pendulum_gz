#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


class LqrControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("lqr_controller_node")

        self.declare_parameter("k", [2.0, 0.3, 35.0, 3.0])
        self.declare_parameter("voltage_limit", 10.0)
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("publish_zero_if_state_missing", True)
        self.declare_parameter("arm_joint_name", "arm_joint")
        self.declare_parameter("pole_joint_name", "pole_joint")
        self.declare_parameter("pole_initial_offset_deg", 0.0)
        self.declare_parameter("arm_ref_rad", 0.0)
        self.declare_parameter("pole_ref_rad", 0.0)
        self.declare_parameter("lqr_enable_pole_angle_deg", 20.0)
        self.declare_parameter("lqr_enable_pole_vel_rad_s", 8.0)
        self.declare_parameter("lqr_disable_pole_angle_deg", 75.0)
        self.declare_parameter("lqr_disable_pole_vel_rad_s", 16.0)
        self.declare_parameter("lqr_stay_on_capture", True)
        self.declare_parameter("velocity_filter_alpha", 0.85)
        self.declare_parameter("voltage_slew_rate_v_per_s", 60.0)
        self.declare_parameter("enable_swing_up", True)
        self.declare_parameter("swing_up_gain", 5.0)
        self.declare_parameter("swing_up_damping", 0.5)
        self.declare_parameter("pendulum_mass", 0.15)
        self.declare_parameter("pendulum_com_length", 0.5)

        self.k = [float(v) for v in self.get_parameter("k").value]
        if len(self.k) != 4:
            raise ValueError("Parameter 'k' must have exactly 4 gains: [k_arm, k_arm_vel, k_pole, k_pole_vel]")

        self.voltage_limit = float(self.get_parameter("voltage_limit").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.publish_zero_if_state_missing = bool(self.get_parameter("publish_zero_if_state_missing").value)
        self.arm_joint_name = str(self.get_parameter("arm_joint_name").value)
        self.pole_joint_name = str(self.get_parameter("pole_joint_name").value)
        self.pole_offset = float(self.get_parameter("pole_initial_offset_deg").value) * 3.141592653589793 / 180.0
        self.arm_ref = float(self.get_parameter("arm_ref_rad").value)
        self.pole_ref = float(self.get_parameter("pole_ref_rad").value)
        self.lqr_enable_pole_angle = (
            float(self.get_parameter("lqr_enable_pole_angle_deg").value) * 3.141592653589793 / 180.0
        )
        self.lqr_enable_pole_vel = float(self.get_parameter("lqr_enable_pole_vel_rad_s").value)
        self.lqr_disable_pole_angle = (
            float(self.get_parameter("lqr_disable_pole_angle_deg").value) * 3.141592653589793 / 180.0
        )
        self.lqr_disable_pole_vel = float(self.get_parameter("lqr_disable_pole_vel_rad_s").value)
        self.lqr_stay_on_capture = bool(self.get_parameter("lqr_stay_on_capture").value)
        self.vel_alpha = float(self.get_parameter("velocity_filter_alpha").value)
        self.voltage_slew_rate = float(self.get_parameter("voltage_slew_rate_v_per_s").value)
        self.enable_swing_up = bool(self.get_parameter("enable_swing_up").value)
        self.swing_up_gain = float(self.get_parameter("swing_up_gain").value)
        self.swing_up_damping = float(self.get_parameter("swing_up_damping").value)
        self.pendulum_mass = float(self.get_parameter("pendulum_mass").value)
        self.pendulum_com_length = float(self.get_parameter("pendulum_com_length").value)

        self.arm_angle = None
        self.arm_vel = None
        self.pole_angle = None
        self.pole_vel = None
        self.arm_vel_filt = 0.0
        self.pole_vel_filt = 0.0
        self._vel_filter_initialized = False
        self.last_u = 0.0
        self.last_u_unsat = 0.0
        self.last_mode = "WAIT_STATE"
        self.last_control_time = time.monotonic()
        self._state_ready_logged = False
        self.lqr_active = False

        self.voltage_cmd_pub = self.create_publisher(Float64, "/rotary_pendulum/motor_voltage_cmd", 10)
        self.mode_pub = self.create_publisher(String, "/rotary_pendulum/control_mode", 10)

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

    @staticmethod
    def wrap_to_pi(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

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
        now = time.monotonic()
        dt = now - self.last_control_time
        if dt <= 1e-6:
            dt = 1.0 / self.control_rate_hz
        self.last_control_time = now

        if None in (self.arm_angle, self.arm_vel, self.pole_angle, self.pole_vel):
            if self.publish_zero_if_state_missing:
                msg = Float64()
                msg.data = 0.0
                self.voltage_cmd_pub.publish(msg)
                self.last_u = 0.0
                self.last_u_unsat = 0.0
            self.last_mode = "WAIT_STATE"
            return

        if not self._vel_filter_initialized:
            self.arm_vel_filt = self.arm_vel
            self.pole_vel_filt = self.pole_vel
            self._vel_filter_initialized = True
        else:
            a = max(0.0, min(0.999, self.vel_alpha))
            self.arm_vel_filt = a * self.arm_vel_filt + (1.0 - a) * self.arm_vel
            self.pole_vel_filt = a * self.pole_vel_filt + (1.0 - a) * self.pole_vel

        arm_err = self.wrap_to_pi(self.arm_angle - self.arm_ref)
        pole_err = self.wrap_to_pi(self.pole_angle - self.pole_ref)

        enter_lqr = (abs(pole_err) <= self.lqr_enable_pole_angle) and (abs(self.pole_vel_filt) <= self.lqr_enable_pole_vel)
        exit_lqr = (abs(pole_err) >= self.lqr_disable_pole_angle) or (abs(self.pole_vel_filt) >= self.lqr_disable_pole_vel)

        if not self.lqr_active and enter_lqr:
            self.lqr_active = True
        elif self.lqr_active and (not self.lqr_stay_on_capture) and exit_lqr:
            self.lqr_active = False

        if self.lqr_active:
            x = [arm_err, self.arm_vel_filt, pole_err, self.pole_vel_filt]
            u = -sum(k_i * x_i for k_i, x_i in zip(self.k, x))
            self.last_mode = "LQR"
        else:
            if self.enable_swing_up:
                m = self.pendulum_mass
                l = self.pendulum_com_length
                g = 9.81
                # Upright reference energy is zero with this definition.
                energy = 0.5 * m * l * l * (self.pole_vel_filt ** 2) + m * g * l * (1.0 - math.cos(pole_err))
                direction_signal = self.pole_vel_filt * math.cos(pole_err)
                direction = 1.0 if direction_signal >= 0.0 else -1.0
                u = -self.swing_up_gain * energy * direction - self.swing_up_damping * self.arm_vel_filt
                self.last_mode = "SWING_UP"
            else:
                u = 0.0
                self.last_mode = "HOLD"

        self.last_u_unsat = u

        if u > self.voltage_limit:
            u = self.voltage_limit
        elif u < -self.voltage_limit:
            u = -self.voltage_limit

        du_max = max(0.0, self.voltage_slew_rate) * dt
        du = u - self.last_u
        if du > du_max:
            u = self.last_u + du_max
        elif du < -du_max:
            u = self.last_u - du_max

        msg = Float64()
        msg.data = u
        self.voltage_cmd_pub.publish(msg)
        self.last_u = u

        mode_msg = String()
        mode_msg.data = self.last_mode
        self.mode_pub.publish(mode_msg)

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
            return

        self.get_logger().info(
            f"mode={self.last_mode}, u={self.last_u:.3f}V (raw={self.last_u_unsat:.3f}V), "
            f"arm={self.arm_angle:.3f}, pole={self.pole_angle:.3f}, lqr_active={self.lqr_active}, "
            f"arm_vf={self.arm_vel_filt:.3f}, pole_vf={self.pole_vel_filt:.3f}"
        )


def main() -> None:
    rclpy.init()
    node = LqrControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
