#!/usr/bin/env python3

from collections import deque
import math
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class StatePlotterNode(Node):
    def __init__(self) -> None:
        super().__init__("state_plotter_node")

        self.declare_parameter("window_seconds", 15.0)
        self.declare_parameter("update_hz", 20.0)

        self.window_seconds = float(self.get_parameter("window_seconds").value)
        self.update_hz = float(self.get_parameter("update_hz").value)

        self.t0 = time.time()
        self.ts = deque()
        self.arm_angle = deque()
        self.arm_vel = deque()
        self.pole_angle = deque()
        self.pole_vel = deque()
        self.motor_voltage = deque()

        self.last_arm_angle = math.nan
        self.last_arm_vel = math.nan
        self.last_pole_angle = math.nan
        self.last_pole_vel = math.nan
        self.last_motor_voltage = math.nan

        self.lock = threading.Lock()

        self.create_subscription(Float64, "/rotary_pendulum/arm_angle", self.arm_angle_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/arm_angular_velocity", self.arm_vel_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/pole_angle", self.pole_angle_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/pole_angular_velocity", self.pole_vel_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/motor_voltage", self.motor_voltage_cb, 20)

        self.create_timer(1.0 / self.update_hz, self.sample)

        self.fig, self.axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        self.fig.suptitle("Rotary Pendulum States")

        self.line_arm_angle, = self.axes[0].plot([], [], label="arm angle [rad]")
        self.line_pole_angle, = self.axes[0].plot([], [], label="pole angle [rad]")
        self.axes[0].set_ylabel("angle [rad]")
        self.axes[0].grid(True)
        self.axes[0].legend(loc="upper right")

        self.line_arm_vel, = self.axes[1].plot([], [], label="arm vel [rad/s]")
        self.line_pole_vel, = self.axes[1].plot([], [], label="pole vel [rad/s]")
        self.axes[1].set_ylabel("angular vel [rad/s]")
        self.axes[1].grid(True)
        self.axes[1].legend(loc="upper right")

        self.line_voltage, = self.axes[2].plot([], [], label="motor voltage [V]")
        self.axes[2].set_ylabel("voltage [V]")
        self.axes[2].set_xlabel("time [s]")
        self.axes[2].grid(True)
        self.axes[2].legend(loc="upper right")

        self.anim = FuncAnimation(self.fig, self.animate, interval=50, blit=False)

    def arm_angle_cb(self, msg: Float64) -> None:
        with self.lock:
            self.last_arm_angle = float(msg.data)

    def arm_vel_cb(self, msg: Float64) -> None:
        with self.lock:
            self.last_arm_vel = float(msg.data)

    def pole_angle_cb(self, msg: Float64) -> None:
        with self.lock:
            self.last_pole_angle = float(msg.data)

    def pole_vel_cb(self, msg: Float64) -> None:
        with self.lock:
            self.last_pole_vel = float(msg.data)

    def motor_voltage_cb(self, msg: Float64) -> None:
        with self.lock:
            self.last_motor_voltage = float(msg.data)

    def sample(self) -> None:
        now = time.time() - self.t0
        with self.lock:
            self.ts.append(now)
            self.arm_angle.append(self.last_arm_angle)
            self.arm_vel.append(self.last_arm_vel)
            self.pole_angle.append(self.last_pole_angle)
            self.pole_vel.append(self.last_pole_vel)
            self.motor_voltage.append(self.last_motor_voltage)

            while self.ts and (now - self.ts[0]) > self.window_seconds:
                self.ts.popleft()
                self.arm_angle.popleft()
                self.arm_vel.popleft()
                self.pole_angle.popleft()
                self.pole_vel.popleft()
                self.motor_voltage.popleft()

    @staticmethod
    def _set_axis_limits(axis, x, y, y_margin=0.1) -> None:
        if not x:
            return
        y = [v for v in y if not math.isnan(v)]
        if not y:
            return

        xmin = x[0]
        xmax = x[-1] if x[-1] > x[0] else x[0] + 1.0
        axis.set_xlim(xmin, xmax)

        ymin = min(y)
        ymax = max(y)
        if abs(ymax - ymin) < 1e-6:
            ymin -= 1.0
            ymax += 1.0
        pad = (ymax - ymin) * y_margin
        axis.set_ylim(ymin - pad, ymax + pad)

    def animate(self, _frame):
        with self.lock:
            t = list(self.ts)
            arm_a = list(self.arm_angle)
            pole_a = list(self.pole_angle)
            arm_v = list(self.arm_vel)
            pole_v = list(self.pole_vel)
            v = list(self.motor_voltage)

        self.line_arm_angle.set_data(t, arm_a)
        self.line_pole_angle.set_data(t, pole_a)
        self.line_arm_vel.set_data(t, arm_v)
        self.line_pole_vel.set_data(t, pole_v)
        self.line_voltage.set_data(t, v)

        self._set_axis_limits(self.axes[0], t, arm_a + pole_a)
        self._set_axis_limits(self.axes[1], t, arm_v + pole_v)
        self._set_axis_limits(self.axes[2], t, v)

        return (
            self.line_arm_angle,
            self.line_pole_angle,
            self.line_arm_vel,
            self.line_pole_vel,
            self.line_voltage,
        )


def main() -> None:
    rclpy.init()
    node = StatePlotterNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        plt.show()
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
