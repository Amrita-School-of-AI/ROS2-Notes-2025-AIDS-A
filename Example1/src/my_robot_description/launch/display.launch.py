import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_robot_description'
    urdf_file_name = 'my_robot.urdf'

    # Get the path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name)

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher: Publishes the TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf_path]),

        # Joint State Publisher GUI: UI to control joints
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui'),

        # RViz2: Visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])