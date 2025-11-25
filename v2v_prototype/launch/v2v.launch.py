from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v2v_prototype',
            executable='bsm_publisher',
            name='vehicle_1',
            parameters=[{'vehicle_id': 1}]
        ),
        Node(
            package='v2v_prototype',
            executable='bsm_listener',
            name='vehicle_2',
            parameters=[{'vehicle_id': 2}]
        )
    ])
