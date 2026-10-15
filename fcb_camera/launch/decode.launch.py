"""Turn ~/image_raw/ffmpeg back into raw images, e.g. while playing a bag.

    ros2 launch fcb_camera decode.launch.py
    ros2 bag play <bag>
    ros2 run rqt_image_view rqt_image_view /fcb_camera/decoded

The decoder is named outright. Left to choose, ffmpeg_image_transport tries
h264_cuvid first, and without CUDA that fails and nothing decodes at all.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input", default_value="/fcb_camera/image_raw",
                              description="base topic; /ffmpeg is appended"),
        DeclareLaunchArgument("output", default_value="/fcb_camera/decoded"),
        DeclareLaunchArgument("decoder", default_value="h264",
                              description="libavcodec decoder, e.g. h264, "
                                          "hevc, h264_cuvid with CUDA"),
        Node(
            package="image_transport",
            executable="republish",
            name="fcb_decoder",
            arguments=["ffmpeg", "raw"],
            remappings=[
                ("in/ffmpeg", [LaunchConfiguration("input"), "/ffmpeg"]),
                ("out", LaunchConfiguration("output")),
            ],
            parameters=[{
                "in.ffmpeg.decoders.h264": LaunchConfiguration("decoder"),
                "in.ffmpeg.decoders.hevc": "hevc",
            }],
        ),
    ])
