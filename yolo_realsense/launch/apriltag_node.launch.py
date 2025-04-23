from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[{
                'tag_family': 'tag36h11',
                'tag_ids': [0],
                'tag_sizes': [0.80],  # meters
            }],
            remappings=[
                ('/image_rect', '/camera/color/image_raw'),
                ('/camera_info', '/camera/color/camera_info'),
            ]
        )
    ])
