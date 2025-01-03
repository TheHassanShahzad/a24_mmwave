import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'a24_mmwave'
    this_dir = get_package_share_directory(package_name)
    
    # Declare a launch argument for specifying the world file
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='home.world',
        description='Name of the world file to load'
    )

    # Use LaunchConfiguration to reference the world file dynamically
    world_name = LaunchConfiguration('world')
    world_file = PathJoinSubstitution([this_dir, 'worlds', world_name])

    gazebo_models_path = os.path.join(this_dir, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    gazebo_params_file = os.path.join(this_dir, 'gazebo', 'gazebo_params.yaml')
    rviz_config_file = os.path.join(this_dir, 'rviz', 'basic.rviz')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )]
        ),
        launch_arguments={'use_sim_time': 'true', 'xacro_file_name': 'a24_mmwave_sim.xacro'}.items()
    )

    # Include the Gazebo launch file, with the world file specified
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )]
        ),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'a24_mmwave'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Add RViz2 node with the custom display configuration
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        # rviz2
    ])
