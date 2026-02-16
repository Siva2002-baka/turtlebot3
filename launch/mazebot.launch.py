import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_path = get_package_share_directory('turtlebot3')
    pkg_worlds = get_package_share_directory('my_gazebo_worlds') 

    world_file = os.path.join(pkg_worlds, 'worlds', 'small_maze.sdf')

    gz_sim_path = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_path, 'urdf', 'turtlebot3_burger.xacro')
    robot_description = Command(['xacro ', urdf_path])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time,'frame_prefix': ''}]
    )
    
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    # 2. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_path, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. Spawn Robot (after Gazebo starts)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'turtlebot3', 
            '-x', '-4.0', 
            '-y', '4.0', 
            '-z', '0.2'
        ],
        output='screen',
    )

    # 4. Bridge node with clock enabled
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # arguments=[
        #     '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        #     '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        #     '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        #     '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        #     '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

        #     '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        #     '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        #     '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        # ],

        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': PathJoinSubstitution([FindPackageShare('turtlebot3'), 'config', 'localization_params_online_async.yaml']),
            'use_sim_time': use_sim_time,
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'True',
            # 'map_subscribe_transient_local': 'true',
            'params_file': PathJoinSubstitution([
                    FindPackageShare('turtlebot3'),
                    'config',
                    'nav2_params.yaml'
        ]),
        }.items()
    )


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', os.path.join(
                        get_package_share_directory('turtlebot3'), 'config', 'turtlebot3.rviz')]
        )

    return LaunchDescription([
        use_sim_time_arg,
        
        # Group 1: Core infrastructure
        gz_sim,
        robot_state_publisher,
        bridge,
        static_transform_publisher,
        # Group 2: Robot spawning
        spawn_robot,


        # Group 4: Intelligence
        TimerAction(period=12.0, actions=[
            slam_toolbox,
            nav2,
            rviz_node,
        ])
        
    ])