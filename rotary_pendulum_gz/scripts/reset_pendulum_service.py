#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl


class ResetPendulumService(Node):
    def __init__(self) -> None:
        super().__init__("reset_pendulum_service")

        self.declare_parameter("world_control_service", "/world/default/control")
        world_service = str(self.get_parameter("world_control_service").value)

        self.gz_control_client = self.create_client(ControlWorld, world_service)
        self.reset_srv = self.create_service(
            Trigger,
            "/rotary_pendulum/reset_initial_angles",
            self.reset_cb,
        )

    def reset_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        if not self.gz_control_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "Gazebo world control service is not available"
            return response

        gz_req = ControlWorld.Request()
        gz_req.world_control = WorldControl()
        gz_req.world_control.pause = False
        # Do not use reset.all here because dynamically spawned entities can disappear.
        gz_req.world_control.reset.model_only = True

        future = self.gz_control_client.call_async(gz_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            response.success = False
            response.message = "Failed to call Gazebo world control reset"
            return response

        if not future.result().success:
            response.success = False
            response.message = "Gazebo reported reset failure"
            return response

        response.success = True
        response.message = "Pendulum state reset to initial angles (arm=0 deg, pole=3 deg)"
        return response


def main() -> None:
    rclpy.init()
    node = ResetPendulumService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
