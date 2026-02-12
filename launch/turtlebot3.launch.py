import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('turtlebot3')
    gz_sim_path = get_package_share_directory('ros_gz_sim')
    urdf_path = os.path.join(pkg_path, 'urdf', 'turtlebot3_burger.xacro')
    robot_description = Command(['xacro ', urdf_path])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 2. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_path, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Spawn Robot (after Gazebo starts)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'turtlebot3', '-z', '0.5'],
        output='screen',
    )

    # 4. Bridge node with clock enabled
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        TimerAction(period=4.0, actions=[bridge]),
    ])