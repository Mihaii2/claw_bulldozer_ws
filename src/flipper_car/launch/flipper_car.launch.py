import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_flipper_car = get_package_share_directory('flipper_car')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg_flipper_car, 'urdf', 'flipper_car.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    world_path = os.path.join(pkg_flipper_car, 'worlds', 'slow_world.sdf')
    rviz_config_path = os.path.join(pkg_flipper_car, 'rviz', 'flipper_car.rviz')
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    # 1. Gazebo Sim with slow-motion physics
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. Spawn Robot (Spawn tilted around Pitch Y-axis at -1.45 rad for testing wheelie balance)
    # Change -P to 0.0 if you want it to drop flat on all 4 wheels first.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'flipper_car', '-topic', 'robot_description', '-z', '0.06', '-P', '-1.45'],
        output='screen'
    )

    # 4. Parameter Bridge (Includes /imu for self-balancing)
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
        package='flipper_car',
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
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        trajectory_tracker_node,
        rviz_node
    ])