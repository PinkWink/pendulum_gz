#!/usr/bin/env python3

import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl


class ResetPendulumService(Node):
    def __init__(self) -> None:
        super().__init__("reset_pendulum_service")

        self.declare_parameter("world_control_service", "/world/default/control")
        self.declare_parameter("model_name", "rotary_inverted_pendulum")
        self.declare_parameter("spawn_z", 0.0)
        self.declare_parameter("reset_timeout_sec", 3.0)
        world_service = str(self.get_parameter("world_control_service").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.spawn_z = float(self.get_parameter("spawn_z").value)
        self.reset_timeout_sec = float(self.get_parameter("reset_timeout_sec").value)

        self.gz_control_client = self.create_client(ControlWorld, world_service)
        self.reset_srv = self.create_service(
            Trigger,
            "/rotary_pendulum/reset_initial_angles",
            self.reset_cb,
        )

    def _run_cmd(self, cmd: list[str], timeout_sec: float) -> tuple[bool, str]:
        try:
            completed = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout_sec,
                check=False,
            )
        except Exception as exc:  # pylint: disable=broad-exception-caught
            return False, str(exc)

        if completed.returncode != 0:
            return False, (completed.stderr or completed.stdout or "command failed").strip()
        return True, (completed.stdout or "").strip()

    def reset_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        if not self.gz_control_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "Gazebo world control service is not available"
            return response

        gz_req = ControlWorld.Request()
        gz_req.world_control = WorldControl()
        gz_req.world_control.pause = True
        # Reset world, then explicitly spawn the model and controllers back.
        gz_req.world_control.reset.all = True

        future = self.gz_control_client.call_async(gz_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.reset_timeout_sec)

        if future.result() is None:
            response.success = False
            response.message = "Failed to call Gazebo world control reset"
            return response

        if not future.result().success:
            response.success = False
            response.message = "Gazebo reported reset failure"
            return response

        spawn_cmd = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-name",
            self.model_name,
            "-topic",
            "robot_description",
            "-z",
            str(self.spawn_z),
        ]
        ok, out = self._run_cmd(spawn_cmd, timeout_sec=10.0)
        if not ok:
            response.success = False
            response.message = f"Reset done, but respawn failed: {out}"
            return response

        jsb_cmd = [
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
        ok, out = self._run_cmd(jsb_cmd, timeout_sec=10.0)
        if not ok:
            response.success = False
            response.message = f"Respawned, but joint_state_broadcaster failed: {out}"
            return response

        arm_ctrl_cmd = [
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "arm_effort_controller",
            "--controller-manager",
            "/controller_manager",
        ]
        ok, out = self._run_cmd(arm_ctrl_cmd, timeout_sec=10.0)
        if not ok:
            response.success = False
            response.message = f"Respawned, but arm_effort_controller failed: {out}"
            return response

        unpause_req = ControlWorld.Request()
        unpause_req.world_control = WorldControl()
        unpause_req.world_control.pause = False
        self.gz_control_client.call_async(unpause_req)

        response.success = True
        response.message = "Pendulum reset complete (model respawned with arm=0 deg, pole=3 deg)"
        return response


def main() -> None:
    rclpy.init()
    node = ResetPendulumService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
