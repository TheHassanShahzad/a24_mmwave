import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = 'a24_mmwave'
    this_dir = get_package_share_directory(package_name)
    
    rviz_config_file = os.path.join(this_dir, 'rviz', 'basic.rviz')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )]
        ),
        launch_arguments={'use_sim_time': 'false', 'xacro_file_name' : 'a24_mmwave_real.xacro'}.items()
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("a24_mmwave"), "urdf", "a24_mmwave_real.xacro"]
            ),
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            os.path.join(this_dir, "config", "controller.yaml"),
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    remapper_node = Node(
        package='a24_mmwave',
        executable='remapper',
        output='screen',
    )

    translator_node = Node(
        package='a24_mmwave',
        executable='translator',
        output='screen',
    )

    ang_to_pwm_node = Node(
        package='a24_mmwave',
        executable='ang_to_pwm',
        output='screen',
    )

    return LaunchDescription([
        rsp,
        controller_manager,
        diff_drive_spawner,
        joint_broad_spawner,
        remapper_node,
        translator_node,
        ang_to_pwm_node,
    ])
