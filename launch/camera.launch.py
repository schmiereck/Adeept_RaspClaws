from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                #'video_device': '/dev/video0',
                'video_device': '/dev/video13',  # ← ISP statt Raw-Sensor!
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'camera_frame_id': 'camera_link',
                'output_encoding': 'rgb8',
                'io_method': 'mmap',  # Wichtig: Memory-mapped I/O statt read
            }]
        )
    ])
