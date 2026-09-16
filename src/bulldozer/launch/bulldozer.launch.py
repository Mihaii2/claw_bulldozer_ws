import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bulldozer = get_package_share_directory('bulldozer')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg_bulldozer, 'urdf', 'bulldozer.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    world_path = os.path.join(pkg_bulldozer, 'worlds', 'slow_world.sdf')
    rviz_config_path = os.path.join(pkg_bulldozer, 'rviz', 'bulldozer.rviz')
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Robot State Publisher (Set use_sim_time: False to prevent startup race condition)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
    )

    # 3. Spawn Robot directly from URDF file (-file avoids topic synchronization lockups)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'bulldozer', '-file', urdf_file, '-z', '0.08'],
        output='screen'
    )

    # 4. Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 5. Trajectory Tracker Node
    trajectory_tracker_node = Node(
        package='bulldozer',
        executable='trajectory_tracker',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. RViz 2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        trajectory_tracker_node,
        rviz_node
    ])