"""
Start the search and rescue system: world, robots and our nodes.

Generalised from the course file multi_robot_challenge_23/launch/rescue_robots_w1.launch.py.
The five course files (w1-w5) only differ in the world file, the map file and the spawn
poses, so this file has one table (SPAWN_POSES) instead of five copies.

Examples (see the README for all arguments):
    ros2 launch rescue_team rescue.launch.py variant:=w3
    ros2 launch rescue_team rescue.launch.py variant:=w1 gui:=true rviz:=true
    ros2 launch rescue_team rescue.launch.py start_world:=false     # only our nodes
    ros2 launch rescue_team rescue.launch.py leader:=false          # drive by hand-sent goals
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

COURSE_PKG = 'multi_robot_challenge_23'

# Robot namespaces in right-of-way order. Passed to every node as the `robots` parameter,
# so the node code never contains a robot name.
ROBOTS = ['tb3_0', 'tb3_1']

# Spawn pose (x, y, yaw) of each robot in ROBOTS order, copied from the course launch files.
SPAWN_POSES = {
    'w1': [(0.0, -1.0, 0.0), (0.0, 1.0, 0.0)],
    'w2': [(-1.0, 1.0, 0.0), (-1.0, -1.0, 0.0)],
    'w3': [(-1.0, 0.5, 0.0), (-1.0, -1.5, 0.0)],
    'w4': [(-0.5, 0.7, 0.0), (-0.5, -1.3, 0.0)],
    'w5': [(0.0, -0.5, 0.0), (0.0, -2.5, 0.0)],
}

# Seconds before scoring starts with scoring:=true. Scoring reads the robots' start poses,
# so it must start after both robots have spawned (about 10 s on the course VM).
SCORING_DELAY = 20.0


def launch_world(context):
    """
    Gazebo, map server and both robots for the chosen variant (only if start_world:=true).

    An OpaqueFunction runs when the launch arguments are known, so plain Python can turn
    the variant name ('w1') into file paths and spawn poses.
    """
    if LaunchConfiguration('start_world').perform(context) != 'true':
        return []

    variant = LaunchConfiguration('variant').perform(context)
    if variant not in SPAWN_POSES:
        raise RuntimeError('variant must be one of {}, got {!r}'.format(
            ', '.join(sorted(SPAWN_POSES)), variant))

    course = get_package_share_directory(COURSE_PKG)
    world_file = os.path.join(course, 'worlds', 'dat160_{}.world'.format(variant))
    map_file = os.path.join(course, 'maps', 'map_dat160_{}.yaml'.format(variant))
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gazebo server, plus the window (gzclient) only if gui:=true.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'gui': LaunchConfiguration('gui')}.items(),
    )

    # The known map of the walls, published once on /map.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'topic_name': 'map', 'frame_id': 'map'}],
    )

    # map_server is a lifecycle node: it publishes nothing until it is activated.
    # The lifecycle manager activates it at start (autostart).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    )

    # One robot per namespace, through the course spawn file. It also starts the robot's
    # robot_state_publisher, aruco_node and the static map -> <robot>/odom transform.
    robots = []
    for name, (x, y, yaw) in zip(ROBOTS, SPAWN_POSES[variant]):
        robots.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                course, 'launch', 'spawn_robot.launch.py')),
            launch_arguments={'namespace': name, 'x': str(x), 'y': str(y),
                              'yaw': str(yaw)}.items(),
        ))

    return [gazebo, map_server, lifecycle_manager] + robots


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('rescue_team'), 'config', 'params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Every one of our nodes gets params.yaml, then the robots list and use_sim_time on top.
    node_params = [params_file, {'robots': ROBOTS, 'use_sim_time': use_sim_time}]

    arguments = [
        DeclareLaunchArgument('variant', default_value='w1',
                              description='World variant: w1, w2, w3, w4 or w5'),
        DeclareLaunchArgument('start_world', default_value='true',
                              description='Start Gazebo, the map server and both robots'),
        DeclareLaunchArgument('scoring', default_value='false',
                              description='Also start the scoring node (not in the competition)'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Show the Gazebo window (slows the simulation)'),
        DeclareLaunchArgument('leader', default_value='true',
                              description='Start the leader; false = only hand-sent goals'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo clock. Leave true'),
    ]

    # One robot_handler in each robot namespace (/tb3_0/robot_handler, /tb3_1/robot_handler).
    robot_handlers = [
        Node(package='rescue_team', executable='robot_handler', name='robot_handler',
             namespace=name, output='screen', parameters=node_params)
        for name in ROBOTS
    ]

    # The global nodes run once, without a namespace.
    marker_detector = Node(
        package='rescue_team', executable='marker_detector', name='marker_detector',
        output='screen', parameters=node_params)
    leader = Node(
        package='rescue_team', executable='leader', name='leader',
        output='screen', parameters=node_params,
        condition=IfCondition(LaunchConfiguration('leader')))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory(COURSE_PKG), 'rviz', 'model.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    # Scoring waits until the robots have spawned (see SCORING_DELAY).
    scoring = TimerAction(
        period=SCORING_DELAY,
        actions=[Node(package='scoring', executable='scoring', name='scoring',
                      output='screen')],
        condition=IfCondition(LaunchConfiguration('scoring')))

    return LaunchDescription(
        arguments
        + [OpaqueFunction(function=launch_world)]
        + robot_handlers
        + [marker_detector, leader, rviz, scoring])
