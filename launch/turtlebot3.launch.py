import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('turtlebot3')
    gz_sim_path = get_package_share_directory('ros_gz_sim')
    controller_config = os.path.join(
        pkg_path,
        'config',
        'ros2_controllers.yaml'
    )
    
    # 1. Robot State Publisher
    urdf_path = os.path.join(pkg_path, 'urdf', 'turtlebot3_burger.urdf')
    robot_description = Command(['xacro ', urdf_path])
    
    node_robot_state_publisher = Node(
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

    # 3. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'turtlebot3', '-z', '0.5'],
        output='screen'
    )

    # 4. Bridge node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   ],
        output='screen'
    )

    # 5. ros2_control Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", controller_config],
    )

    # 6. Teleop Node
    teleop_node = Node(
        package='turtlebot3',
        executable='teleop_node',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_spawner],
            )
        ),

        TimerAction(period=10.0, actions=[teleop_node]),
    ])