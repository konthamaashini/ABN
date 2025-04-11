import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Package name and robot name
    namePackage = 'robot'
    robotXacroName = 'abn_robot'

    # File paths
    pathModelFile = os.path.join(get_package_share_directory(namePackage), 'urdf', 'abn_robot.urdf.xacro')

    # Process xacro to URDF string
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Set GAZEBO_PLUGIN_PATH if needed (optional for your setup)
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join('/usr/lib/x86_64-linux-gnu/gazebo-11/plugins')
    )

    # Include the Gazebo launch file from gazebo_ros
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch
    )

    # Spawn the robot using the robot_description parameter
    spawnModelNode = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )

    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # Joint State Publisher Node (required for joint visualization in Gazebo)
    jointStatePublisherNode = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Create the launch description and add all actions
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(set_gazebo_plugin_path)
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(jointStatePublisherNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    return launchDescriptionObject
