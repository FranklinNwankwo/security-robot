import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

PERSON_CLASS_ID = 0  # 'person' in the COCO dataset YOLO is trained on

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


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        self.declare_parameter('model_path', str(Path.home() / 'models' / 'yolov8n.pt'))
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('inference_rate_hz', 3.0)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('cpu_threads', 2)
        self.declare_parameter('publish_annotated', True)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        rate_hz = float(self.get_parameter('inference_rate_hz').value)
        image_topic = self.get_parameter('image_topic').value

        if not Path(model_path).exists():
            raise FileNotFoundError(
                f'YOLO model not found at {model_path}. Download it with: '
                f"mkdir -p ~/models && cd ~/models && python3 -c \"from ultralytics import YOLO; YOLO('yolov8n.pt')\""
            )

        # Limit PyTorch's CPU threads so the simulation keeps its share of the CPU
        import torch
        torch.set_num_threads(int(self.get_parameter('cpu_threads').value))
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model {model_path}')

        self.latest_image = None
        self.latest_is_new = False

        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.detections_pub = self.create_publisher(Detection2DArray, '/perception/person_detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/perception/annotated_image', 10)

        # Run inference on a timer, always on the newest frame, instead of on every frame
        self.create_timer(1.0 / rate_hz, self.process_latest)

        self.frames_processed = 0
        self.total_inference_s = 0.0
        self.last_report = time.monotonic()
        self.last_count = 0

        self.get_logger().info(
            f'Listening on {image_topic}, inference at up to {rate_hz:.1f} Hz, '
            f'confidence >= {self.conf_threshold:.2f}'
        )

    def image_callback(self, msg):
        self.latest_image = msg
        self.latest_is_new = True

    def process_latest(self):
        if not self.latest_is_new or self.latest_image is None:
            return
        msg = self.latest_image
        self.latest_is_new = False

        try:
            frame = image_msg_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        start = time.monotonic()
        result = self.model.predict(
            frame,
            imgsz=self.image_size,
            conf=self.conf_threshold,
            classes=[PERSON_CLASS_ID],
            verbose=False,
        )[0]
        self.total_inference_s += time.monotonic() - start
        self.frames_processed += 1

        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().numpy() if len(boxes) else np.zeros((0, 4))
        confs = boxes.conf.cpu().numpy() if len(boxes) else np.zeros((0,))

        detections = Detection2DArray()
        detections.header = msg.header

        for i, ((x1, y1, x2, y2), conf) in enumerate(zip(xyxy, confs)):
            det = Detection2D()
            det.header = msg.header
            det.id = f'person_{i}'
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = float(conf)
            det.results.append(hypothesis)
            detections.detections.append(det)

            if self.publish_annotated:
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
                cv2.putText(frame, f'person {conf:.2f}', (p1[0], max(p1[1] - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.detections_pub.publish(detections)
        if self.publish_annotated:
            self.annotated_pub.publish(bgr_to_image_msg(frame, msg.header))

        count = len(detections.detections)
        if count != self.last_count:
            if count:
                self.get_logger().info(f'{count} person(s) in view, best confidence {confs.max():.2f}')
            else:
                self.get_logger().info('No person in view')
            self.last_count = count

        now = time.monotonic()
        if now - self.last_report > 10.0:
            avg_ms = 1000.0 * self.total_inference_s / max(self.frames_processed, 1)
            self.get_logger().info(
                f'[stats] {self.frames_processed} frames processed, avg inference {avg_ms:.0f} ms'
            )
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PersonDetector()
    except Exception as e:
        rclpy.logging.get_logger('person_detector').error(str(e))
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
