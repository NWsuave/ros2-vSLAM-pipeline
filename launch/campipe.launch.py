from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Publishes a static transform: base_link -> camera_link
        # Edit child frame if your image header uses a different frame_id.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),

        # Converts /image_raw (YUY2) -> /image_raw/bgr8 (BGR8)
        Node(
            package='yuy2_to_bgr',
            executable='yuy2_to_bgr_node',
            name='yuy2_to_bgr',
            output='screen'
        )
    ])
