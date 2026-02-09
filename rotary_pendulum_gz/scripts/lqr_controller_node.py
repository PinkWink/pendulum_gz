#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Float64, String


class LqrControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("lqr_controller_node")

        self.declare_parameter("k", [14.0, 2.5, 180.0, 24.0])
        self.declare_parameter("swing_up_voltage_limit", 24.0)
        self.declare_parameter("lqr_voltage_limit", 24.0)
        self.declare_parameter("control_rate_hz", 150.0)
        self.declare_parameter("publish_zero_if_state_missing", True)

        self.declare_parameter("arm_ref_rad", 0.0)
        self.declare_parameter("pole_ref_rad", 0.0)

        self.declare_parameter("lqr_enter_pole_angle_deg", 5.0)
        self.declare_parameter("lqr_enter_pole_vel_rad_s", 1.2)
        self.declare_parameter("lqr_exit_pole_angle_deg", 10.0)

        self.declare_parameter("swingup_rearm_pole_angle_deg", 1.0)
        self.declare_parameter("swingup_rearm_pole_vel_rad_s", 0.15)
        self.declare_parameter("min_swing_up_time_sec", 0.8)

        self.declare_parameter("velocity_filter_alpha", 0.75)
        self.declare_parameter("voltage_slew_rate_v_per_s", 400.0)

        self.declare_parameter("swing_up_gain", 26.0)
        self.declare_parameter("swing_up_damping", 0.25)
        self.declare_parameter("swing_up_arm_centering_gain", 4.0)
        self.declare_parameter("swing_up_min_voltage", 10.0)
        self.declare_parameter("swing_up_pump_voltage", 14.0)

        self.declare_parameter("swing_up_arm_limit_deg", 80.0)
        self.declare_parameter("swing_up_arm_limit_recovery_gain", 28.0)

        self.declare_parameter("swing_up_upright_brake_angle_deg", 20.0)
        self.declare_parameter("swing_up_upright_brake_gain", 3.5)

        self.declare_parameter("pendulum_mass", 0.35)
        self.declare_parameter("pendulum_com_length", 0.55)

        self.k = [float(v) for v in self.get_parameter("k").value]
        if len(self.k) != 4:
            raise ValueError("Parameter 'k' must have exactly 4 gains: [k_arm, k_arm_vel, k_pole, k_pole_vel]")

        self.swing_up_voltage_limit = float(self.get_parameter("swing_up_voltage_limit").value)
        self.lqr_voltage_limit = float(self.get_parameter("lqr_voltage_limit").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.publish_zero_if_state_missing = bool(self.get_parameter("publish_zero_if_state_missing").value)

        self.arm_ref = float(self.get_parameter("arm_ref_rad").value)
        self.pole_ref = float(self.get_parameter("pole_ref_rad").value)

        self.lqr_enter_pole_angle = math.radians(float(self.get_parameter("lqr_enter_pole_angle_deg").value))
        self.lqr_enter_pole_vel = float(self.get_parameter("lqr_enter_pole_vel_rad_s").value)
        self.lqr_exit_pole_angle = math.radians(float(self.get_parameter("lqr_exit_pole_angle_deg").value))

        self.swingup_rearm_pole_angle = math.radians(float(self.get_parameter("swingup_rearm_pole_angle_deg").value))
        self.swingup_rearm_pole_vel = float(self.get_parameter("swingup_rearm_pole_vel_rad_s").value)
        self.min_swing_up_time = float(self.get_parameter("min_swing_up_time_sec").value)

        self.vel_alpha = float(self.get_parameter("velocity_filter_alpha").value)
        self.voltage_slew_rate = float(self.get_parameter("voltage_slew_rate_v_per_s").value)

        self.swing_up_gain = float(self.get_parameter("swing_up_gain").value)
        self.swing_up_damping = float(self.get_parameter("swing_up_damping").value)
        self.swing_up_arm_centering_gain = float(self.get_parameter("swing_up_arm_centering_gain").value)
        self.swing_up_min_voltage = float(self.get_parameter("swing_up_min_voltage").value)
        self.swing_up_pump_voltage = float(self.get_parameter("swing_up_pump_voltage").value)

        self.swing_up_arm_limit = math.radians(float(self.get_parameter("swing_up_arm_limit_deg").value))
        self.swing_up_arm_limit_recovery_gain = float(self.get_parameter("swing_up_arm_limit_recovery_gain").value)

        self.swing_up_upright_brake_angle = math.radians(
            float(self.get_parameter("swing_up_upright_brake_angle_deg").value)
        )
        self.swing_up_upright_brake_gain = float(self.get_parameter("swing_up_upright_brake_gain").value)

        self.pendulum_mass = float(self.get_parameter("pendulum_mass").value)
        self.pendulum_com_length = float(self.get_parameter("pendulum_com_length").value)

        self.arm_angle = None
        self.arm_vel = None
        self.pole_angle = None
        self.pole_vel = None
        self.seen_arm_angle = False
        self.seen_arm_vel = False
        self.seen_pole_angle = False
        self.seen_pole_vel = False

        self.arm_vel_filt = 0.0
        self.pole_vel_filt = 0.0
        self._vel_filter_initialized = False

        self.last_u = 0.0
        self.last_u_unsat = 0.0
        self.last_mode = "WAIT_STATE"
        self.last_control_time = time.monotonic()
        self._state_ready_logged = False
        self._last_mode_reported = "WAIT_STATE"

        self.lqr_active = False
        self.swingup_armed = True
        self.start_time = time.monotonic()

        self.voltage_cmd_pub = self.create_publisher(Float64, "/rotary_pendulum/motor_voltage_cmd", 10)
        self.mode_pub = self.create_publisher(String, "/rotary_pendulum/control_mode", 10)

        self.create_subscription(Float64, "/rotary_pendulum/arm_angle", self.arm_angle_cb, 10)
        self.create_subscription(Float64, "/rotary_pendulum/arm_angular_velocity", self.arm_vel_cb, 10)
        self.create_subscription(Float64, "/rotary_pendulum/pole_angle", self.pole_angle_cb, 10)
        self.create_subscription(Float64, "/rotary_pendulum/pole_angular_velocity", self.pole_vel_cb, 10)

        wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(1.0 / self.control_rate_hz, self.control_loop, clock=wall_clock)
        self.create_timer(1.0, self.status_log_loop, clock=wall_clock)

        self.get_logger().info(
            "Controller started. "
            f"K={self.k}, enter={math.degrees(self.lqr_enter_pole_angle):.1f}deg, "
            f"exit={math.degrees(self.lqr_exit_pole_angle):.1f}deg, "
            f"Vlim_swing={self.swing_up_voltage_limit}, Vlim_lqr={self.lqr_voltage_limit}"
        )

    @staticmethod
    def wrap_to_pi(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def arm_angle_cb(self, msg: Float64) -> None:
        self.arm_angle = float(msg.data)
        self.seen_arm_angle = True

    def arm_vel_cb(self, msg: Float64) -> None:
        self.arm_vel = float(msg.data)
        self.seen_arm_vel = True

    def pole_angle_cb(self, msg: Float64) -> None:
        self.pole_angle = float(msg.data)
        self.seen_pole_angle = True

    def pole_vel_cb(self, msg: Float64) -> None:
        self.pole_vel = float(msg.data)
        self.seen_pole_vel = True

    def _publish_mode(self) -> None:
        m = String()
        m.data = self.last_mode
        self.mode_pub.publish(m)

    def _publish_u(self, u: float, limit: float, dt: float) -> None:
        self.last_u_unsat = u

        if u > limit:
            u = limit
        elif u < -limit:
            u = -limit

        du_max = max(0.0, self.voltage_slew_rate) * dt
        du = u - self.last_u
        if du > du_max:
            u = self.last_u + du_max
        elif du < -du_max:
            u = self.last_u - du_max

        cmd = Float64()
        cmd.data = u
        self.voltage_cmd_pub.publish(cmd)
        self.last_u = u

    def control_loop(self) -> None:
        now = time.monotonic()
        dt = now - self.last_control_time
        if dt <= 1e-6:
            dt = 1.0 / self.control_rate_hz
        self.last_control_time = now

        if (
            (not self.seen_arm_angle)
            or (not self.seen_arm_vel)
            or (not self.seen_pole_angle)
            or (not self.seen_pole_vel)
            or None in (self.arm_angle, self.arm_vel, self.pole_angle, self.pole_vel)
        ):
            if self.publish_zero_if_state_missing:
                z = Float64()
                z.data = 0.0
                self.voltage_cmd_pub.publish(z)
                self.last_u = 0.0
                self.last_u_unsat = 0.0
            self.last_mode = "WAIT_STATE"
            self._publish_mode()
            return

        if not self._vel_filter_initialized:
            self.arm_vel_filt = self.arm_vel
            self.pole_vel_filt = self.pole_vel
            self._vel_filter_initialized = True
        else:
            a = max(0.0, min(0.999, self.vel_alpha))
            self.arm_vel_filt = a * self.arm_vel_filt + (1.0 - a) * self.arm_vel
            self.pole_vel_filt = a * self.pole_vel_filt + (1.0 - a) * self.pole_vel

        arm_err_wrap = self.wrap_to_pi(self.arm_angle - self.arm_ref)
        arm_err_raw = self.arm_angle - self.arm_ref
        pole_err = self.wrap_to_pi(self.pole_angle - self.pole_ref)

        forced_swing_up = (now - self.start_time) < self.min_swing_up_time

        enter_lqr = (
            (not forced_swing_up)
            and (abs(pole_err) <= self.lqr_enter_pole_angle)
            and (abs(self.pole_vel_filt) <= self.lqr_enter_pole_vel)
        )

        if self.lqr_active:
            if abs(pole_err) > self.lqr_exit_pole_angle:
                self.lqr_active = False
                self.swingup_armed = False
                self.last_mode = "FREE_FALL"
                self._publish_u(0.0, self.lqr_voltage_limit, dt)
            else:
                x = [arm_err_wrap, self.arm_vel_filt, pole_err, self.pole_vel_filt]
                u = -sum(k_i * x_i for k_i, x_i in zip(self.k, x))
                self.last_mode = "LQR"
                self._publish_u(u, self.lqr_voltage_limit, dt)
        else:
            if enter_lqr:
                self.lqr_active = True
                x = [arm_err_wrap, self.arm_vel_filt, pole_err, self.pole_vel_filt]
                u = -sum(k_i * x_i for k_i, x_i in zip(self.k, x))
                self.last_mode = "LQR"
                self._publish_u(u, self.lqr_voltage_limit, dt)
            elif not self.swingup_armed:
                if (
                    abs(pole_err) <= self.swingup_rearm_pole_angle
                    and abs(self.pole_vel_filt) <= self.swingup_rearm_pole_vel
                ):
                    self.swingup_armed = True
                self.last_mode = "FREE_FALL"
                self._publish_u(0.0, self.swing_up_voltage_limit, dt)
            else:
                m = self.pendulum_mass
                l = self.pendulum_com_length
                g = 9.81
                energy = 0.5 * m * l * l * (self.pole_vel_filt ** 2) + m * g * l * (1.0 + math.cos(pole_err))
                energy_error = energy - (2.0 * m * g * l)
                direction = 1.0 if (self.pole_vel_filt * math.cos(pole_err)) >= 0.0 else -1.0

                u = (
                    self.swing_up_gain * energy_error * direction
                    + self.swing_up_pump_voltage * direction
                    - self.swing_up_damping * self.arm_vel_filt
                    - self.swing_up_arm_centering_gain * arm_err_raw
                )

                if abs(pole_err) < self.swing_up_upright_brake_angle:
                    u += -self.swing_up_upright_brake_gain * self.pole_vel_filt

                if abs(u) < self.swing_up_min_voltage and abs(pole_err) > self.lqr_enter_pole_angle:
                    u = self.swing_up_min_voltage * (1.0 if u >= 0.0 else -1.0)

                if abs(arm_err_raw) > self.swing_up_arm_limit:
                    u = (
                        -self.swing_up_arm_limit_recovery_gain * math.copysign(1.0, arm_err_raw)
                        - 2.0 * self.swing_up_damping * self.arm_vel_filt
                    )

                self.last_mode = "SWING_UP"
                self._publish_u(u, self.swing_up_voltage_limit, dt)

        self._publish_mode()
        if self.last_mode != self._last_mode_reported:
            self.get_logger().info(
                f"Mode switched: {self._last_mode_reported} -> {self.last_mode} "
                f"(armed={self.swingup_armed}, arm={self.arm_angle:.3f}, pole={self.pole_angle:.3f}, "
                f"arm_v={self.arm_vel_filt:.3f}, pole_v={self.pole_vel_filt:.3f})"
            )
            self._last_mode_reported = self.last_mode

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
                f"x=[{self.arm_angle:.3f}, {self.arm_vel:.3f}, {self.pole_angle:.3f}, {self.pole_vel:.3f}]"
            )
            self._state_ready_logged = True
            return

        self.get_logger().info(
            f"mode={self.last_mode}, lqr_active={self.lqr_active}, armed={self.swingup_armed}, "
            f"u={self.last_u:.3f}V (raw={self.last_u_unsat:.3f}V), arm={self.arm_angle:.3f}, pole={self.pole_angle:.3f}, "
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
