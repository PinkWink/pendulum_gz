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
                "k": [12.0, 2.0, 160.0, 20.0],
                "swing_up_voltage_limit": 22.0,
                "lqr_voltage_limit": 24.0,
                "control_rate_hz": 100.0,
                "capture_pole_angle_deg": 60.0,
                "capture_pole_vel_rad_s": 14.0,
                "capture_arm_angle_deg": 85.0,
                "capture_arm_vel_rad_s": 15.0,
                "exit_pole_angle_deg": 75.0,
                "exit_pole_vel_rad_s": 18.0,
                "exit_arm_angle_deg": 95.0,
                "exit_arm_vel_rad_s": 12.0,
                "enter_lqr_hold_time_sec": 0.03,
                "exit_lqr_hold_time_sec": 0.25,
                "min_swing_up_time_sec": 2.0,
                "velocity_filter_alpha": 0.85,
                "voltage_slew_rate_v_per_s": 200.0,
                "swing_up_gain": 18.0,
                "swing_up_damping": 0.20,
                "swing_up_arm_centering_gain": 3.0,
                "swing_up_min_voltage": 8.0,
                "swing_up_arm_limit_deg": 80.0,
                "swing_up_arm_limit_recovery_gain": 22.0,
                "swing_up_pump_voltage": 10.0,
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
