#!/usr/bin/env python3
import math
from typing import Dict

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class PendulumInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("pendulum_interface_node")

        self.declare_parameter("arm_joint_name", "arm_joint")
        self.declare_parameter("pole_joint_name", "pole_joint")
        self.declare_parameter("pole_initial_offset_deg", 3.0)
        self.declare_parameter("motor_resistance", 2.0)
        self.declare_parameter("back_emf_constant", 0.02)
        self.declare_parameter("torque_constant", 0.02)
        self.declare_parameter("max_effort", 8.0)

        self.arm_joint = self.get_parameter("arm_joint_name").value
        self.pole_joint = self.get_parameter("pole_joint_name").value
        self.pole_offset = math.radians(self.get_parameter("pole_initial_offset_deg").value)
        self.motor_r = float(self.get_parameter("motor_resistance").value)
        self.motor_ke = float(self.get_parameter("back_emf_constant").value)
        self.motor_kt = float(self.get_parameter("torque_constant").value)
        self.max_effort = float(self.get_parameter("max_effort").value)

        self.last_joint_pos: Dict[str, float] = {}
        self.last_joint_vel: Dict[str, float] = {}
        self.last_voltage = 0.0
        self.last_current = 0.0

        self.arm_angle_pub = self.create_publisher(Float64, "/rotary_pendulum/arm_angle", 10)
        self.pole_angle_pub = self.create_publisher(Float64, "/rotary_pendulum/pole_angle", 10)
        self.arm_vel_pub = self.create_publisher(Float64, "/rotary_pendulum/arm_angular_velocity", 10)
        self.pole_vel_pub = self.create_publisher(Float64, "/rotary_pendulum/pole_angular_velocity", 10)
        self.voltage_pub = self.create_publisher(Float64, "/rotary_pendulum/motor_voltage", 10)
        self.current_pub = self.create_publisher(Float64, "/rotary_pendulum/motor_current", 10)
        self.effort_cmd_pub = self.create_publisher(Float64MultiArray, "/arm_effort_controller/commands", 10)

        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 20)
        self.create_subscription(Float64, "/rotary_pendulum/motor_voltage_cmd", self.voltage_cmd_cb, 10)

        # Publish motor telemetry using wall time so it is available even when sim time is paused.
        self.create_timer(0.02, self.publish_motor_state, clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    def joint_state_cb(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        if self.arm_joint in name_to_index:
            i = name_to_index[self.arm_joint]
            if i < len(msg.position):
                self.last_joint_pos[self.arm_joint] = msg.position[i]
            if i < len(msg.velocity):
                self.last_joint_vel[self.arm_joint] = msg.velocity[i]

        if self.pole_joint in name_to_index:
            i = name_to_index[self.pole_joint]
            if i < len(msg.position):
                self.last_joint_pos[self.pole_joint] = msg.position[i]
            if i < len(msg.velocity):
                self.last_joint_vel[self.pole_joint] = msg.velocity[i]

        if self.arm_joint in self.last_joint_pos:
            arm_msg = Float64()
            arm_msg.data = self.last_joint_pos[self.arm_joint]
            self.arm_angle_pub.publish(arm_msg)

        if self.pole_joint in self.last_joint_pos:
            pole_msg = Float64()
            pole_msg.data = self.last_joint_pos[self.pole_joint] + self.pole_offset
            self.pole_angle_pub.publish(pole_msg)

        if self.arm_joint in self.last_joint_vel:
            arm_vel_msg = Float64()
            arm_vel_msg.data = self.last_joint_vel[self.arm_joint]
            self.arm_vel_pub.publish(arm_vel_msg)

        if self.pole_joint in self.last_joint_vel:
            pole_vel_msg = Float64()
            pole_vel_msg.data = self.last_joint_vel[self.pole_joint]
            self.pole_vel_pub.publish(pole_vel_msg)

    def voltage_cmd_cb(self, msg: Float64) -> None:
        arm_vel = self.last_joint_vel.get(self.arm_joint, 0.0)
        voltage = float(msg.data)

        current = (voltage - (self.motor_ke * arm_vel)) / self.motor_r
        effort = self.motor_kt * current

        effort = max(min(effort, self.max_effort), -self.max_effort)

        effort_msg = Float64MultiArray()
        effort_msg.data = [effort]
        self.effort_cmd_pub.publish(effort_msg)

        self.last_voltage = voltage
        self.last_current = current

    def publish_motor_state(self) -> None:
        v_msg = Float64()
        v_msg.data = self.last_voltage
        self.voltage_pub.publish(v_msg)

        i_msg = Float64()
        i_msg.data = self.last_current
        self.current_pub.publish(i_msg)


def main() -> None:
    rclpy.init()
    node = PendulumInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
