from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # v4l2_camera node - publishes /image_raw
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                #'video_device': '/dev/video0',
                # Du brauchst ein Device mit :capture: UND YUYV-Unterstützung:
                # Device        Typ                 YUYV?           Empfehlung
                # /dev/video0   unicam capture      ✓               ❌ Raw-Sensor (funktioniert nicht)
                # /dev/video14  bcm2835-isp capture ✓ RGB3, YUYV    ✅ BESTE WAHL
                # /dev/video15  bcm2835-isp capture ✓ YUYV          ✅ Alternative
                # /dev/video21  bcm2835-isp capture ✓ RGB3, YUYV    ✅ Alternative
                # /dev/video22  bcm2835-isp capture ✓ YUYV          ✅ Alternative
                'video_device': '/dev/video14',  # ← ISP statt Raw-Sensor!
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'camera_frame_id': 'camera_link',
                'output_encoding': 'rgb8',
                'io_method': 'mmap',  # Wichtig: Memory-mapped I/O statt read
            }],
            remappings=[
                ('/image_raw', '/image_raw'),
                ('/camera_info', '/camera_info'),
            ]
        ),

        # image_transport republish node - subscribes /image_raw, publishes /image_raw/compressed
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',
            output='screen',
            arguments=['raw', 'compressed'],
            parameters=[{
                'compressed.format': 'jpeg',
                'compressed.jpeg_quality': 85
            }],
            remappings=[
                ('in', '/image_raw'),
                ('out', '/image_raw')
            ]
        )
    ])


