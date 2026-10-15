"""ROS 2 driver for the Sony FCB-EV9520L: video out, mode and zoom in.

What fcb_view.py does, as a node instead of a window. The camera is found
and opened the same way, through the same fcb_base_driver code, so the
board quirks handled there (repeated frames, padded VISCA replies, a link
that needs rebuilding) are handled here too.

    ~/image_raw          sensor_msgs/Image, bgr8, rotated like fcb_view.py
    ~/image_raw/ffmpeg   ffmpeg_image_transport_msgs/FFMPEGPacket, H.264 --
                         record this one: ~1 MB/s against ~370 for raw
    ~/zoom               std_msgs/Float32, current magnification, 5 Hz
    ~/imaging_mode       std_msgs/String, rgb / ir / rgb+ir / auto (latched)

    ~/set_imaging_mode   fcb_interfaces/srv/SetImagingMode
    ~/set_zoom           fcb_interfaces/srv/SetZoom

    ros2 service call /fcb_camera/set_imaging_mode \\
        fcb_interfaces/srv/SetImagingMode "{mode: ir}"
    ros2 service call /fcb_camera/set_zoom \\
        fcb_interfaces/srv/SetZoom "{ratio: 7.5, wait: true}"

Each video topic is only produced while something subscribes to it, so a
bag recording just the ffmpeg topic costs no raw conversions.

Only one program may hold the camera: stop fcb_view.py / fcb_record.py
before starting this, or it finds the devices busy.
"""
import array
import math
import threading
import time

import cv2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from fcb_base_driver import devices, visca, zoom_map
from fcb_base_driver.frame_grabber import FrameGrabber
from fcb_base_driver.visca_link import ViscaError, ViscaLink, ViscaTimeout
from fcb_base_driver.zoom_servo import FINE_BAND, ZoomServo
from fcb_camera.ffmpeg_encoder import PacketEncoder
from fcb_interfaces.srv import SetImagingMode, SetZoom

#: CAM_ICR name -> what the services and topics call it. The same four
#: names fcb_view.py and the flight recorder use.
MODE_LABEL = {
    "day": "rgb",
    "night": "ir",
    "night_color": "rgb+ir",
    "auto": "auto",
}
#: Spellings visca.normalize_icr_mode does not know, on top of its own.
EXTRA_ALIASES = {"rgb+ir": "night_color", "rgb_ir": "night_color",
                 "rgbir": "night_color"}

#: How close the lens has to be to a target, in position counts, for a
#: waiting set_zoom to call it arrived. Twice what the servo lands within.
ARRIVED_BAND = 2 * FINE_BAND

#: Seconds without a frame before saying so, and how often to repeat it.
STALL_WARN = 3.0

#: The camera's nominal rate: the encoder's rate-control hint when
#: ffmpeg_fps does not set one. Headers carry the real capture times.
NOMINAL_FPS = 60.0


def to_bgr(frame):
    """Three-channel BGR from whatever FrameGrabber handed over.

    FrameGrabber already decodes raw YUV 4:2:2; this covers the backends
    that hand back grey or BGRA instead.
    """
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


class FcbCameraNode(Node):

    def __init__(self):
        super().__init__("fcb_camera")
        self.declare_parameter("video", "")
        self.declare_parameter("port", "")
        self.declare_parameter("baud", 0)
        self.declare_parameter("address", 1)
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("fourcc", "")
        self.declare_parameter("rotate", 180)
        self.declare_parameter("publish_fps", 0.0)
        self.declare_parameter("image_scale", 1.0)
        self.declare_parameter("frame_id", "fcb_camera_optical_frame")
        self.declare_parameter("icr", "")
        # H.264 for rosbag. Any libavcodec encoder: libx264 on CPU (the
        # Orin's ffmpeg has no NVENC), h264_nvenc on a desktop GPU, libx265
        # for smaller files at more CPU.
        self.declare_parameter("ffmpeg", True)
        self.declare_parameter("ffmpeg_encoder", "libx264")
        self.declare_parameter("ffmpeg_bit_rate", 8_000_000)
        self.declare_parameter("ffmpeg_gop", 0)
        self.declare_parameter("ffmpeg_options",
                               "preset=ultrafast,tune=zerolatency")
        self.declare_parameter("ffmpeg_fps", 0.0)
        self.declare_parameter("ffmpeg_scale", 1.0)

        def param(name):
            return self.get_parameter(name).value

        self.address = param("address")
        self.rotate = param("rotate")
        self.publish_period = (1.0 / param("publish_fps")
                               if param("publish_fps") > 0 else 0.0)
        self.image_scale = param("image_scale")
        self.frame_id = param("frame_id")
        self.ffmpeg = {name: param("ffmpeg_" + name) for name in (
            "encoder", "bit_rate", "gop", "options", "fps", "scale")}

        self.link = None
        self.servo = None
        self.grabber = None
        self.mode = None
        self.control_baud = None
        self._mode_lock = threading.Lock()
        self._running = True

        group = ReentrantCallbackGroup()
        self.image_pub = self.create_publisher(Image, "~/image_raw", 1)
        self.zoom_pub = self.create_publisher(Float32, "~/zoom", 10)
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, "~/imaging_mode", latched)
        self.ffmpeg_pub = None
        if param("ffmpeg"):
            # Reliable and a few deep: a bag missing one packet loses video
            # until the next keyframe, not just a frame.
            self.ffmpeg_pub = self.create_publisher(
                FFMPEGPacket, "~/image_raw/ffmpeg", 10)

        self._describe_board()
        self.open_control(param("port") or None, param("baud") or None)
        if param("icr"):
            ok, message = self.apply_mode(param("icr"))
            if not ok:
                self.get_logger().error(f"startup imaging mode: {message}")
        self.open_camera(param("video") or None, param("width"),
                         param("height"), param("fourcc") or None)

        # Reentrant, so a set_zoom that waits for the lens does not hold up
        # set_imaging_mode or the status timer behind it.
        self.create_service(SetImagingMode, "~/set_imaging_mode",
                            self.on_set_imaging_mode, callback_group=group)
        self.create_service(SetZoom, "~/set_zoom", self.on_set_zoom,
                            callback_group=group)
        self.create_timer(0.2, self.publish_status, callback_group=group)

        targets = [self._publish_frames]
        if self.ffmpeg_pub is not None:
            targets.append(self._encode_frames)
        self._threads = [threading.Thread(target=self._quietly, args=(target,),
                                          daemon=True) for target in targets]
        for thread in self._threads:
            thread.start()

    def _quietly(self, target):
        """Run a video thread; at shutdown, let it end without a traceback.

        Ctrl-C invalidates the ROS context before close() can stop these
        threads, so their next publish raises. That is the end of the run,
        not a fault; anything raised while still running is.
        """
        try:
            target()
        except Exception:
            if self._running and rclpy.ok():
                raise

    # -- setup -----------------------------------------------------------

    def _describe_board(self):
        """Name the board before opening it; says so if it is on USB 2."""
        status, lines = devices.describe_camera()
        for line in lines:
            # rclpy pins each log call site to one severity, so two calls.
            if status == 0:
                self.get_logger().info(f"board: {line}")
            else:
                self.get_logger().warning(f"board: {line}")

    def make_link(self, port=None, baud=None):
        """A working VISCA link, or None. Also the servo's reconnect factory.

        On a reconnect only the camera board's own ports are swept, at the
        baud that answered before -- the full sweep would write VISCA into
        whatever else is plugged in, every two seconds, while the camera is
        away.
        """
        if port is not None:
            ports = [port]
        elif self.control_baud is not None:
            ports = devices.camera_serial_ports()
            if not ports:
                return None
        else:
            ports = None
        bauds = [baud] if baud else (
            [self.control_baud] if self.control_baud else None)
        port, baud = devices.autodetect_visca(ports, bauds, self.address)
        if port is None:
            return None
        link = None
        try:
            link = ViscaLink(port, baud=baud, address=self.address)
            link.if_clear()
        except Exception as exc:
            self.get_logger().warning(f"VISCA: {port} at {baud} baud would not "
                                   f"open: {exc}")
            # Or its exclusive lock outlives it, and every later reconnect
            # finds the port "locked by another process".
            if link is not None:
                try:
                    link.close()
                except Exception:
                    pass
            return None
        self.control_baud = baud
        self.get_logger().info(f"VISCA: {port} at {baud} baud")
        return link

    def open_control(self, port, baud):
        """VISCA, for zoom and the IR cut filter. Optional, as in fcb_view.py:
        without it the video still publishes and the services say why they
        cannot act."""
        self.link = self.make_link(port, baud)
        if self.link is None:
            self.get_logger().error(
                "no VISCA port answered -- set_zoom and set_imaging_mode "
                "will fail. The video still publishes.")
            return
        self.servo = ZoomServo(
            self.link, rc=None, link_factory=self.make_link,
            on_status=lambda msg: self.get_logger().warning(msg))
        try:
            ratio = self.servo.sync_from_camera()
            self.get_logger().info(f"zoom: currently {ratio:.1f}x")
        except Exception as exc:
            self.get_logger().warning(f"could not read the zoom position: {exc}")
        self.read_mode()

    def read_mode(self):
        """Report the mode the camera is already in, without changing it.

        The FCB keeps it across a power cycle. The inquiry reports where the
        filter is, not how it got there, so AUTO reads back as whichever
        mode it has picked.
        """
        try:
            reply = self.servo.link.inquiry(visca.icr_mode_inq(self.address))
            mode = visca.parse_icr_mode(reply)
        except (ViscaError, ViscaTimeout, ValueError) as exc:
            self.get_logger().warning(f"could not read the imaging mode: {exc}")
            return
        self._set_mode_state(mode)
        self.get_logger().info(f"imaging mode: {MODE_LABEL[mode]} (as found)")

    def open_camera(self, device, width, height, fourcc):
        if device is None:
            device = devices.autodetect_video(
                log=lambda msg: self.get_logger().info(f"video: {msg}"))
        if device is None:
            self.get_logger().error(
                "no FCB camera found -- pass video:=/dev/videoN if it is "
                "attached under a name this does not recognise. No video "
                "will publish; the services still work.")
            return
        capture = devices.open_capture(device, width, height, fourcc)
        if capture is None:
            self.get_logger().error(f"could not open {device}")
            return
        pixel_format = devices.fourcc_of(capture)
        yuv_code = devices.yuv422_to_bgr_code(pixel_format)
        repeats = devices.usb_id(device) in devices.REPEATING_FRAME_USB_IDS
        self.get_logger().info(
            f"video: {device} at {int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
            f"{pixel_format or 'format not reported'}"
            + (", dropping the board's repeated frames" if repeats else ""))
        self.grabber = FrameGrabber(capture, yuv_code, drop_repeats=repeats)

    # -- video -----------------------------------------------------------

    def _frames(self, min_period=0.0):
        """Yield (frame, captured_at) for each new frame, at most once per
        `min_period` seconds. Warns when the camera goes quiet."""
        seq = 0
        last_frame_at = time.monotonic()
        last_warned = 0.0
        last_yielded = 0.0
        while self._running and rclpy.ok():
            frame, new_seq, captured_at = self.grabber.wait_for_frame(seq, 1.0)
            now = time.monotonic()
            if frame is None:
                if (now - last_frame_at > STALL_WARN
                        and now - last_warned > STALL_WARN * 5):
                    last_warned = now
                    self.get_logger().warning(
                        f"no frames for {now - last_frame_at:.0f}s -- check "
                        "the USB 3 link, or power-cycle the camera board")
                continue
            seq = new_seq
            last_frame_at = now
            if min_period and now - last_yielded < min_period:
                continue
            last_yielded = now
            yield frame, captured_at

    def _publish_frames(self):
        """Raw images, on their own thread so services never wait."""
        if self.grabber is None:
            return
        for frame, captured_at in self._frames(self.publish_period):
            # 6 MB to convert and copy per frame: not for nobody.
            if self.image_pub.get_subscription_count() == 0:
                continue
            self.image_pub.publish(self._to_msg(frame, captured_at))

    def _prepare(self, frame, scale, even=False):
        """BGR, rotated and scaled. `even` rounds the size down to even
        numbers, which 4:2:0 encoders need."""
        frame = to_bgr(frame)
        if self.rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        if scale != 1.0 or even:
            height, width = frame.shape[:2]
            size = (int(width * scale), int(height * scale))
            if even:
                size = (size[0] & ~1, size[1] & ~1)
            if size != (width, height):
                frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
        return frame

    def _stamp(self, captured_at):
        """ROS time of a frame, from the monotonic time it was dequeued --
        not the time it got around to being published."""
        age = time.monotonic() - captured_at
        return self.get_clock().now().nanoseconds - int(age * 1e9)

    @staticmethod
    def _set_header(msg, stamp, frame_id):
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = frame_id

    def _to_msg(self, frame, captured_at):
        frame = self._prepare(frame, self.image_scale)
        msg = Image()
        self._set_header(msg, self._stamp(captured_at), self.frame_id)
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        # An array.array is taken as-is; anything else is checked element by
        # element, which for 6 MB takes longer than the next frame does.
        msg.data = array.array("B", frame.tobytes())
        return msg

    # -- ffmpeg ----------------------------------------------------------

    def _encode_frames(self):
        """H.264 packets for ~/image_raw/ffmpeg, while anyone listens.

        The encoder is dropped when the last subscriber leaves and a fresh
        one made when the next arrives, so every subscriber -- a bag
        started mid-session included -- begins on a keyframe carrying the
        stream headers, and can decode from its first packet.
        """
        if self.grabber is None:
            return
        opts = self.ffmpeg
        period = 1.0 / opts["fps"] if opts["fps"] > 0 else 0.0
        encoder = None
        for frame, captured_at in self._frames(period):
            if self.ffmpeg_pub.get_subscription_count() == 0:
                encoder = None
                continue
            frame = self._prepare(frame, opts["scale"], even=True)
            height, width = frame.shape[:2]
            if encoder is None or (encoder.context.width,
                                   encoder.context.height) != (width, height):
                try:
                    encoder = PacketEncoder(
                        opts["encoder"], width, height,
                        opts["fps"] or NOMINAL_FPS, opts["bit_rate"],
                        opts["gop"], opts["options"])
                except Exception as exc:
                    # A missing python3-av or an encoder this ffmpeg lacks
                    # costs the compressed topic, never the node.
                    self.get_logger().error(
                        f"ffmpeg: cannot encode with {opts['encoder']}: "
                        f"{type(exc).__name__}: {exc} -- ~/image_raw/ffmpeg "
                        "is off for this run")
                    return
                self.get_logger().info(
                    f"ffmpeg: {opts['encoder']} {width}x{height} at "
                    f"{opts['bit_rate'] / 1e6:g} Mbit/s, "
                    f"keyframe every {encoder.context.gop_size} frames")
            try:
                packets = encoder.encode(frame, self._stamp(captured_at))
            except Exception as exc:
                self.get_logger().error(f"ffmpeg: encode failed: {exc}; "
                                        "restarting the encoder")
                encoder = None
                continue
            for pts, keyframe, payload, stamp in packets:
                msg = FFMPEGPacket()
                self._set_header(msg, stamp, self.frame_id)
                msg.width, msg.height = width, height
                msg.encoding = encoder.encoding
                msg.pts = pts
                msg.flags = 1 if keyframe else 0
                msg.is_bigendian = False
                msg.data = array.array("B", payload)
                self.ffmpeg_pub.publish(msg)

    # -- status ----------------------------------------------------------

    def _set_mode_state(self, mode):
        self.mode = mode
        self.mode_pub.publish(String(data=MODE_LABEL[mode]))

    def publish_status(self):
        if self.servo is not None and self.servo.ratio is not None:
            self.zoom_pub.publish(Float32(data=float(self.servo.ratio)))

    # -- imaging mode ----------------------------------------------------

    def apply_mode(self, requested):
        """(ok, message) after switching the IR cut filter."""
        if self.servo is None:
            return False, "no VISCA link to the camera"
        key = str(requested).strip().lower()
        try:
            mode = visca.normalize_icr_mode(EXTRA_ALIASES.get(key, key))
        except ValueError:
            return False, (f"unknown mode {requested!r}; use rgb, ir, "
                           "rgb+ir or auto")
        with self._mode_lock:
            # Once more on failure: over USB the VISCA link loses the odd
            # reply while video streams, and the command usually took anyway.
            error = None
            for _ in range(2):
                try:
                    self.servo.set_icr(mode)
                    error = None
                    break
                except Exception as exc:
                    # Any exception, not just VISCA's: a serial error from an
                    # unplugged board escaping here would take the node down.
                    error = exc
            if error is not None:
                return False, f"{MODE_LABEL[mode]} would not take: {error}"
            self._set_mode_state(mode)
        message = f"imaging mode {MODE_LABEL[mode]}"
        self.get_logger().info(message)
        return True, message

    def on_set_imaging_mode(self, request, response):
        ok, message = self.apply_mode(request.mode)
        if not ok:
            self.get_logger().error(message)
        response.success = ok
        response.message = message
        response.mode = MODE_LABEL.get(self.mode, "")
        return response

    # -- zoom ------------------------------------------------------------

    def on_set_zoom(self, request, response):
        servo = self.servo
        response.ratio = float(servo.ratio or 0.0) if servo else 0.0
        if servo is None:
            response.success = False
            response.message = "no VISCA link to the camera"
            return response
        if not math.isfinite(request.ratio):
            response.success = False
            response.message = f"not a magnification: {request.ratio}"
            return response

        # float(): clamped to the table's ends it comes back an int, which
        # a float32 field refuses.
        target = float(servo.set_ratio(request.ratio))
        response.target_ratio = target
        message = f"zoom {target:.2f}x"
        if abs(target - request.ratio) > 1e-3:
            message += (f" (asked {request.ratio:.2f}x; the lens does "
                        f"{zoom_map.MIN_RATIO:.0f}-{zoom_map.MAX_RATIO:.0f}x)")
        self.get_logger().info(message)

        response.success = True
        if request.wait:
            timeout = request.timeout_s if request.timeout_s > 0 else 10.0
            outcome = self._wait_for_lens(target, timeout)
            if outcome != "arrived":
                response.success = False
            message += {
                "arrived": ", arrived",
                "superseded": ", superseded by a newer request",
                "timeout": f", not there after {timeout:.1f}s",
            }[outcome]
        response.ratio = float(servo.ratio or 0.0)
        response.message = message
        return response

    def _wait_for_lens(self, target, timeout):
        """'arrived', 'superseded' or 'timeout': the lens settling on `target`.

        Settled means two polls in a row on target with the servo not
        driving -- a single one can catch the poll that read the position
        just before the servo decided to move.
        """
        goal = zoom_map.ratio_to_position(target)
        deadline = time.monotonic() + timeout
        settled = 0
        while time.monotonic() < deadline and self._running:
            servo = self.servo
            if servo.manual_ratio != target:
                return "superseded"
            on_target = (servo.position is not None
                         and abs(servo.position - goal) <= ARRIVED_BAND
                         and not servo.is_driving)
            settled = settled + 1 if on_target else 0
            if settled >= 2:
                return "arrived"
            time.sleep(0.15)
        return "timeout"

    # -- shutdown --------------------------------------------------------

    def close(self):
        self._running = False
        if self.grabber is not None:
            self.grabber.stop()
        if self.servo is not None:
            self.servo.stop()
            try:
                self.servo.link.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = FcbCameraNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
