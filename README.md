# fcb_ros2_humble_driver

ROS 2 Humble driver for the **Sony FCB-EV9520L** block camera on a USB
interface board (Twiga **USB3 NeoHD** or **Oppila** LVDS-USB3). It publishes
the video, raw and H.264-compressed for rosbag, and gives two services: one
switches **RGB / IR**, one sets the **zoom**.

The camera handling (finding the board, VISCA, the zoom servo, the board
quirks) is the same code as `fcb_view.py` / `fcb_record.py` in
[fcb_orin](https://github.com/kreideprinze/fcb_orin), vendored in
`fcb_camera/fcb_base_driver/`.

| package | |
|---|---|
| `fcb_interfaces` | the service types `SetImagingMode` and `SetZoom` |
| `fcb_camera` | the driver node, plus launch files to run and to decode |

## Build

    sudo apt install ros-humble-ffmpeg-image-transport python3-av \
        python3-serial python3-opencv v4l-utils
    cd ~/ros2_ws/src
    git clone https://github.com/kreideprinze/fcb_ros2_humble_driver.git
    cd .. && colcon build --packages-select fcb_interfaces fcb_camera
    source install/setup.bash

(`rosdep install --from-paths src -y --ignore-src` installs the same.)

## Run

    ros2 launch fcb_camera fcb_camera.launch.py

The video device and the VISCA port are found automatically; to set them
yourself, pass `video:=/dev/video0 port:=/dev/ttyACM0`. The user running it
needs to be in the `dialout` and `video` groups. In Docker, use
`--group-add` for both. Without `dialout` no VISCA port answers, and both
services refuse.

## Services

Switch RGB / IR:

    ros2 service call /fcb_camera/set_imaging_mode fcb_interfaces/srv/SetImagingMode "{mode: ir}"

| `mode` | what the camera does |
|---|---|
| `rgb` | daylight colour, IR cut filter in (also `day`) |
| `ir` | filter out, infrared-sensitive mono (also `night`) |
| `rgb+ir` | filter out, colour kept (also `night_color`) |
| `auto` | the camera switches on scene brightness |

Zoom, to an exact magnification from 1x to 30x:

    ros2 service call /fcb_camera/set_zoom fcb_interfaces/srv/SetZoom "{ratio: 7.5}"
    ros2 service call /fcb_camera/set_zoom fcb_interfaces/srv/SetZoom "{ratio: 7.5, wait: true}"

With `wait: true` the reply comes only once the lens has arrived, or after
`timeout_s` (default 10 s), and gives the position it actually reached. A
value outside 1-30x is clamped, and the reply says so. A newer request
replaces one still in progress.

Measured on a NeoHD: 1x to 7.3x in 3.5 s, landing on 7.33x; 30x to 1x in
6.2 s. A failed call returns `success: false` with the reason, and the
node keeps running.

## Topics

| topic | type | |
|---|---|---|
| `/fcb_camera/image_raw` | `sensor_msgs/Image` | bgr8, rotated 180° (camera mounted upside down; `rotate:=0` to turn off) |
| `/fcb_camera/image_raw/ffmpeg` | `ffmpeg_image_transport_msgs/FFMPEGPacket` | H.264, **record this one** |
| `/fcb_camera/zoom` | `std_msgs/Float32` | current magnification, 5 Hz |
| `/fcb_camera/imaging_mode` | `std_msgs/String` | latched: as found at startup, then on every change |

Each video topic is produced only while something subscribes to it.

## Recording a small rosbag

Raw 1080p is about **170 MB/s** in a bag (measured), and Python could only
publish about 29 of the 60 frames a second. The ffmpeg topic carries every
frame as H.264 at **about 1.1 MB/s** at the default 8 Mbit/s, roughly 150
times smaller:

    ros2 bag record /fcb_camera/image_raw/ffmpeg /fcb_camera/zoom /fcb_camera/imaging_mode

| recorded, 1080p | rate in the bag | size |
|---|---|---|
| `image_raw` | 29 fps | 428 MB in 2.5 s |
| `image_raw/ffmpeg` | 60 fps | 11 MB in 9.8 s |

Encoding is done in the node by libavcodec (through PyAV). Each packet uses
the format `ffmpeg_image_transport` writes itself, so the standard plugin
decodes it. Every new subscriber, including a bag started mid-session,
starts on a keyframe. Packets carry the frame's capture time.

To watch it, live or from a bag:

    ros2 launch fcb_camera decode.launch.py     # publishes /fcb_camera/decoded
    ros2 bag play <bag>
    ros2 run rqt_image_view rqt_image_view /fcb_camera/decoded

Use the launch file rather than a bare `republish ffmpeg raw`. Without a
named decoder, the plugin tries `h264_cuvid` first, and with no CUDA that
fails and nothing is decoded.

Encoder settings, as launch arguments:

| argument | default | |
|---|---|---|
| `ffmpeg` | `true` | `false` turns the compressed topic off |
| `ffmpeg_encoder` | `libx264` | any libavcodec encoder: `libx265` (smaller, more CPU), `h264_nvenc` (desktop NVIDIA GPU) |
| `ffmpeg_bit_rate` | `8000000` | bits/s; 8 Mbit/s = 1 MB/s of bag |
| `ffmpeg_gop` | `0` | frames between keyframes; 0 = one a second |
| `ffmpeg_options` | `preset=ultrafast,tune=zerolatency` | encoder options, `key=value,...` |
| `ffmpeg_fps` | `0` | cap on encoded frames per second; 0 = every frame |
| `ffmpeg_scale` | `1.0` | e.g. `0.5` encodes 960x540 |

On a Jetson Orin, the ffmpeg from apt has no access to the hardware
encoder, so `libx264` runs on the CPU. If it falls behind at 1080p60,
frames are skipped, not queued, so the bag stays in step with real time.
`ffmpeg_fps:=30` or `ffmpeg_scale:=0.5` lowers the load.

## Things to know

- **Only one program can hold the camera.** Stop `fcb_view.py`, the flight
  recorder, or any other node using the board first, or the devices are
  busy.
- **A NeoHD on a USB 2 link** controls fine, but cannot carry 1080p video.
  The node says so at startup. Use a USB 3 port and cable.
- **Oppila board:** closing its video stream usually leaves it sending
  nothing until its 12 V is power-cycled. Starting and stopping this node
  does exactly that, so start it once and leave it running.
- **"no frames for Ns"** in the log means the board stopped delivering:
  check the USB 3 link, or power-cycle the board.
