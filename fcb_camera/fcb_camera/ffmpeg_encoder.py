"""H.264 / HEVC packets in ffmpeg_image_transport's wire format.

Raw 1080p bgr8 is 6.2 MB a frame -- over 370 MB/s at 60 fps, more than a
rosbag can usefully hold. Encoded at 8 Mbit/s the same stream is 1 MB/s.

The packets are what ffmpeg_image_transport's own encoder publishes, so
anything with that plugin installed decodes them: rqt_image_view, a
`republish ffmpeg raw`, or a bag played back through either. The format
was read off the stock encoder rather than assumed:

    encoding  "h264;yuv420p;bgr8;bgr8"  codec;libav input;cv_bridge;ROS
    pts       a frame counter
    flags     1 on a keyframe (AV_PKT_FLAG_KEY)
    data      Annex B, with SPS/PPS repeated in-band on every keyframe

Encoding runs in-process through PyAV (python3-av), so it is ffmpeg's
libavcodec doing the work without a pipe or a subprocess to babysit.
"""
from fractions import Fraction

#: Encoder name -> the codec name decoders are looked up by. The first
#: field of the encoding string has to be the codec, not the encoder:
#: "libx264" finds no decoder on the other side, "h264" does.
CODEC_OF = {
    "libx264": "h264", "h264_nvenc": "h264", "h264_vaapi": "h264",
    "h264_v4l2m2m": "h264", "h264_qsv": "h264",
    "libx265": "hevc", "hevc_nvenc": "hevc", "hevc_vaapi": "hevc",
    "hevc_qsv": "hevc",
    "libsvtav1": "av1", "libaom-av1": "av1", "av1_nvenc": "av1",
}

#: What the encoder is fed, and what the ROS image it stood in for was.
PIXEL_FORMAT = "yuv420p"
ROS_ENCODING = "bgr8"


def parse_options(text):
    """'preset=ultrafast,tune=zerolatency' -> {'preset': ..., 'tune': ...}"""
    options = {}
    for item in filter(None, (part.strip() for part in text.split(","))):
        key, sep, value = item.partition("=")
        if not sep:
            raise ValueError(f"ffmpeg option {item!r} is not key=value")
        options[key.strip()] = value.strip()
    return options


class PacketEncoder:
    """One encoder for one stream; make a new one to restart at a keyframe."""

    def __init__(self, encoder, width, height, fps, bit_rate, gop_size,
                 options=""):
        import av      # here, so a missing python3-av disables only this

        # 4:2:0 halves both dimensions, so odd ones would be refused.
        if width % 2 or height % 2:
            raise ValueError(f"{width}x{height}: yuv420p needs even sizes")
        rate = max(1, int(round(fps)))
        self.context = av.CodecContext.create(encoder, "w")
        self.context.width = width
        self.context.height = height
        self.context.pix_fmt = PIXEL_FORMAT
        self.context.time_base = Fraction(1, rate)
        self.context.framerate = Fraction(rate, 1)
        self.context.bit_rate = int(bit_rate)
        self.context.gop_size = int(gop_size) if gop_size > 0 else rate
        # No B-frames: each packet then belongs to the frame just sent, so
        # it goes out at once with that frame's timestamp. Set as libavcodec's
        # generic "bf" option: PyAV 8 (Ubuntu 22.04's) has no max_b_frames.
        self.context.options = {"bf": "0", **parse_options(options)}
        self.context.open()
        self.encoding = ";".join([CODEC_OF.get(encoder, encoder),
                                  PIXEL_FORMAT, ROS_ENCODING, ROS_ENCODING])
        self._av = av
        self._pts = 0
        self._stamps = {}

    def encode(self, bgr, stamp):
        """[(pts, is_keyframe, payload bytes, stamp)] for one BGR frame.

        Usually one packet, but an encoder with lookahead may hold frames
        back and return them later, so each packet carries the stamp of the
        frame it actually is.
        """
        frame = self._av.VideoFrame.from_ndarray(bgr, format="bgr24")
        frame.pts = self._pts
        self._stamps[self._pts] = stamp
        self._pts += 1
        out = []
        for packet in self.context.encode(frame):
            pts = packet.pts if packet.pts is not None else self._pts - 1
            out.append((pts, packet.is_keyframe, bytes(packet),
                        self._stamps.pop(pts, stamp)))
        # Anything an encoder never returned is not coming back.
        if len(self._stamps) > 256:
            for old in sorted(self._stamps)[:-256]:
                del self._stamps[old]
        return out
