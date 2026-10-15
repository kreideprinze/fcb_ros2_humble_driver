"""The FCB-EV9520L driver: video plus imaging-mode and zoom services.

    ros2 launch fcb_camera fcb_camera.launch.py
    ros2 launch fcb_camera fcb_camera.launch.py video:=/dev/video0 port:=/dev/ttyACM0

Empty video / port mean autodetect, the same way fcb_view.py finds them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    # name: (default, type, description)
    "video": ("", str, "capture device; empty autodetects the FCB board"),
    "port": ("", str, "VISCA serial port; empty autodetects"),
    "baud": ("0", int, "VISCA baud; 0 tries 9600, 38400, 115200"),
    "width": ("1920", int, "capture width"),
    "height": ("1080", int, "capture height"),
    "rotate": ("180", int, "0 or 180; the camera is mounted upside down"),
    "publish_fps": ("0.0", float, "cap on image_raw rate; 0 publishes every frame"),
    "image_scale": ("1.0", float, "resize image_raw by this factor"),
    "icr": ("", str, "imaging mode to set at startup; empty leaves it as found"),
    "frame_id": ("fcb_camera_optical_frame", str, "header.frame_id of image_raw"),
    "ffmpeg": ("true", bool, "publish H.264 on image_raw/ffmpeg, for rosbag"),
    "ffmpeg_encoder": ("libx264", str, "libavcodec encoder: libx264, libx265, h264_nvenc ..."),
    "ffmpeg_bit_rate": ("8000000", int, "bits per second; 8 Mbit/s is 1 MB/s of bag"),
    "ffmpeg_gop": ("0", int, "frames between keyframes; 0 is one a second"),
    "ffmpeg_options": ("preset=ultrafast,tune=zerolatency", str, "encoder options, key=value,..."),
    "ffmpeg_fps": ("0.0", float, "cap on the encoded rate; 0 encodes every frame"),
    "ffmpeg_scale": ("1.0", float, "resize before encoding, e.g. 0.5 for 960x540"),
}


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default,
                                description=description)
          for name, (default, _, description) in ARGS.items()],
        Node(
            package="fcb_camera",
            executable="camera_node",
            name="fcb_camera",
            output="screen",
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                for name, (_, kind, _) in ARGS.items()
            }],
        ),
    ])
