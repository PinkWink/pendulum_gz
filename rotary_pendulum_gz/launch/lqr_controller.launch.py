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
                "k": [22.0, 4.0, 260.0, 36.0],
                "swing_up_voltage_limit": 24.0,
                "lqr_voltage_limit": 28.0,
                "control_rate_hz": 150.0,
                "lqr_enter_pole_angle_deg": 5.0,
                "lqr_enter_pole_vel_rad_s": 1.2,
                "lqr_exit_pole_angle_deg": 10.0,
                "swingup_rearm_pole_angle_deg": 1.0,
                "swingup_rearm_pole_vel_rad_s": 0.15,
                "min_swing_up_time_sec": 0.8,
                "velocity_filter_alpha": 0.75,
                "voltage_slew_rate_v_per_s": 700.0,
                "swing_up_gain": 26.0,
                "swing_up_damping": 0.25,
                "swing_up_arm_centering_gain": 4.0,
                "swing_up_min_voltage": 10.0,
                "swing_up_pump_voltage": 14.0,
                "swing_up_arm_limit_deg": 80.0,
                "swing_up_arm_limit_recovery_gain": 28.0,
                "swing_up_upright_brake_angle_deg": 20.0,
                "swing_up_upright_brake_gain": 3.5,
                "pendulum_mass": 0.35,
                "pendulum_com_length": 0.55,
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
