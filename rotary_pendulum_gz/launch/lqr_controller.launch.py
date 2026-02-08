from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lqr_node = Node(
        package="rotary_pendulum_gz",
        executable="lqr_controller_node.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "k": [10.0, 2.0, 120.0, 12.0],
                "voltage_limit": 24.0,
                "control_rate_hz": 100.0,
            }
        ],
    )

    state_plotter_node = Node(
        package="rotary_pendulum_gz",
        executable="state_plotter_node.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "window_seconds": 15.0,
                "update_hz": 20.0,
            }
        ],
    )

    return LaunchDescription([lqr_node, state_plotter_node])
