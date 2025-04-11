import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for the URDF path
        DeclareLaunchArgument('robot_description', default_value='file:///home/aashini/ABN/src/robot/urdf/robot.urdf', description="Path to the robot URDF file"),

        # Log information (optional)
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('robot_description', 'file:///home/aashini/ABN/src/robot/urdf/robot.urdf'),
            msg="Launching visualization of robot with URDF from: /home/aashini/ABN/src/robot/urdf/robot.urdf"
        ),

        # Launch the robot state publisher to publish the robot's URDF to ROS
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}],
            remappings=[('/joint_states', '/robot/joint_states')]
        ),

        # Launch the joint state publisher to simulate joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}],
            remappings=[('/joint_states', '/robot/joint_states')]
        ),

        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', '/home/aashini/ABN/src/robot/rviz/robot.rviz']  # Optional: path to a custom RViz configuration file
        ),
    ])
