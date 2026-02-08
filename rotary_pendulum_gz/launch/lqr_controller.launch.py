from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lqr_node = Node(
        package="rotary_pendulum_gz",
        executable="lqr_controller_node.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "k": [2.0, 0.3, 35.0, 3.0],
                "voltage_limit": 10.0,
                "control_rate_hz": 100.0,
                "capture_pole_angle_deg": 15.0,
                "capture_pole_vel_rad_s": 4.0,
                "min_swing_up_time_sec": 2.0,
                "velocity_filter_alpha": 0.85,
                "voltage_slew_rate_v_per_s": 60.0,
                "swing_up_gain": 5.0,
                "swing_up_damping": 0.5,
            }
        ],
    )

    state_plotter_node = Node(
        package="rotary_pendulum_gz",
        executable="state_plotter_node.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "window_seconds": 15.0,
                "update_hz": 20.0,
            }
        ],
    )

    return LaunchDescription([lqr_node, state_plotter_node])
