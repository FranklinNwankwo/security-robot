import cv2
import numpy as np
from sensor_msgs.msg import Image

ENCODING_CHANNELS = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}


def image_msg_to_bgr(msg):
    """Convert a sensor_msgs/Image to an OpenCV BGR array (no cv_bridge needed)."""
    channels = ENCODING_CHANNELS.get(msg.encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == 'bgr8':
        return img.copy()
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    return cv2.cvtColor(img[:, :, 0], cv2.COLOR_GRAY2BGR)


def bgr_to_image_msg(img, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg
