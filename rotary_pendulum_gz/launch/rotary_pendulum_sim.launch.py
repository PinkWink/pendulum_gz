from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("rotary_pendulum_gz")
    world = PathJoinSubstitution([pkg_share, "worlds", "empty.sdf"])
    urdf_xacro = PathJoinSubstitution([pkg_share, "urdf", "rotary_inverted_pendulum.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    robot_description = Command([
        "xacro ",
        urdf_xacro,
        " controllers_file:=",
        controllers,
    ])

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", world],
        output="screen",
    )

    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=["/opt/ros/jazzy/lib", ":", EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value="")],
    )
    ld_library_path = SetEnvironmentVariable(
        name="LD_LIBRARY_PATH",
        value=["/opt/ros/jazzy/lib", ":", EnvironmentVariable("LD_LIBRARY_PATH", default_value="")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "rotary_inverted_pendulum", "-topic", "robot_description", "-z", "0.0"],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/default/model/rotary_inverted_pendulum/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/world/default/control@ros_gz_interfaces/srv/ControlWorld",
        ],
        remappings=[
            ("/world/default/model/rotary_inverted_pendulum/joint_state", "/joint_states"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["arm_effort_controller", "--controller-manager", "/controller_manager"],
    )

    pendulum_interface = Node(
        package="rotary_pendulum_gz",
        executable="pendulum_interface_node.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    reset_service = Node(
        package="rotary_pendulum_gz",
        executable="reset_pendulum_service.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    delayed_spawn = TimerAction(period=2.0, actions=[spawn_robot])
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, arm_effort_controller_spawner],
    )

    return LaunchDescription(
        [
            gz_plugin_path,
            ld_library_path,
            gz_sim,
            robot_state_publisher,
            bridge,
            delayed_spawn,
            delayed_controllers,
            pendulum_interface,
            reset_service,
        ]
    )
