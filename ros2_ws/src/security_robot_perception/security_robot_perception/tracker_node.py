import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import Detection3D, Detection3DArray

from security_robot_perception.image_utils import image_msg_to_bgr, bgr_to_image_msg


# ---------------- geometry helpers ----------------

def det_to_box(det):
    cx = det.bbox.center.position.x
    cy = det.bbox.center.position.y
    w, h = det.bbox.size_x, det.bbox.size_y
    return np.array([cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0], dtype=float)


def box_area(b):
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def intersection(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def iou(a, b):
    inter = intersection(a, b)
    union = box_area(a) + box_area(b) - inter
    return inter / union if union > 0 else 0.0


def containment(small, big):
    area = box_area(small)
    return intersection(small, big) / area if area > 0 else 0.0


def center_distance(a, b):
    ca = ((a[0] + a[2]) / 2.0, (a[1] + a[3]) / 2.0)
    cb = ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)
    return float(np.hypot(ca[0] - cb[0], ca[1] - cb[1]))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def appearance_hist(bgr, box):
    """Hue/saturation histogram of the central body area (shirt and trousers) - a simple clothing fingerprint."""
    img_h, img_w = bgr.shape[:2]
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    xa, xb = int(max(0, x1 + 0.35 * bw)), int(min(img_w, x2 - 0.35 * bw))   # central strip: skip outstretched arms
    ya, yb = int(max(0, y1 + 0.15 * bh)), int(min(img_h, y1 + 0.90 * bh))   # below the head, above the shoes
    if xb - xa < 4 or yb - ya < 8:
        return None
    hsv = cv2.cvtColor(bgr[ya:yb, xa:xb], cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])
    cv2.normalize(hist, hist)
    return hist


def find_visual_tracker_factory():
    """Best available OpenCV single-object tracker: CSRT (contrib) > KCF > MIL (built in)."""
    legacy = getattr(cv2, 'legacy', None)
    candidates = [
        ('CSRT', getattr(cv2, 'TrackerCSRT_create', None)),
        ('CSRT', getattr(legacy, 'TrackerCSRT_create', None) if legacy else None),
        ('KCF', getattr(cv2, 'TrackerKCF_create', None)),
        ('MIL', getattr(cv2, 'TrackerMIL_create', None)),
    ]
    for name, factory in candidates:
        if factory is not None:
            return name, factory
    return None, None


# ---------------- track ----------------

class Track:
    def __init__(self, key, box, conf, yaw, now):
        self.key = key
        self.id = None              # assigned once confirmed
        self.box = box              # image box, valid at robot heading `yaw`
        self.yaw = yaw
        self.conf = conf
        self.hits = 1
        self.first_seen = now
        self.last_seen = now        # last YOLO detection
        self.last_alive = now       # last YOLO detection OR successful visual update
        self.visual = None          # OpenCV tracker following this person between YOLO hits
        self.last_visual_init = -1e9
        self.ref_area = box_area(box)


# ---------------- node ----------------

class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        self.declare_parameter('detections_topic', '/perception/person_detections')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('camera_hfov_rad', 1.2)          # must match the Webots camera fieldOfView
        self.declare_parameter('image_width', 640)
        self.declare_parameter('ego_motion_compensation', True)
        self.declare_parameter('min_hits', 3)                  # YOLO detections needed to confirm a person
        self.declare_parameter('max_age_s', 2.0)               # drop a person after this long with neither YOLO nor visual lock
        self.declare_parameter('tentative_max_age_s', 1.5)     # drop unconfirmed candidates after this
        self.declare_parameter('iou_threshold', 0.2)
        self.declare_parameter('max_center_distance_px', 120.0)
        self.declare_parameter('relink_distance_px', 200.0)
        self.declare_parameter('containment_threshold', 0.7)
        self.declare_parameter('smoothing', 0.5)
        self.declare_parameter('visual_tracking', True)
        self.declare_parameter('visual_scale', 0.5)            # run the visual tracker on a half-size image (faster)
        self.declare_parameter('max_visual_only_s', 5.0)      # YOLO must re-confirm a person at least this often
        self.declare_parameter('visual_reinit_interval_s', 3.0)
        self.declare_parameter('image_buffer_s', 3.0)
        self.declare_parameter('draw_rate_hz', 5.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('reid_radius_m', 1.5)          # same person if found this close to where they were lost
        self.declare_parameter('reid_memory_s', 300.0)        # remember lost people for this long
        self.declare_parameter('bearing_window_rad', 0.04)    # LiDAR rays checked around the person's direction
        self.declare_parameter('person_height_m', 1.7)        # for the size-based distance estimate
        self.declare_parameter('merge_radius_m', 0.6)         # two tracks this close in the room = same person

        gp = self.get_parameter
        self.min_hits = int(gp('min_hits').value)
        self.max_age = float(gp('max_age_s').value)
        self.tentative_max_age = float(gp('tentative_max_age_s').value)
        self.iou_threshold = float(gp('iou_threshold').value)
        self.max_center_px = float(gp('max_center_distance_px').value)
        self.relink_px = float(gp('relink_distance_px').value)
        self.containment_threshold = float(gp('containment_threshold').value)
        self.alpha = float(gp('smoothing').value)
        self.compensate = bool(gp('ego_motion_compensation').value)
        self.scale = float(gp('visual_scale').value)
        self.max_visual_only = float(gp('max_visual_only_s').value)
        self.reinit_interval = float(gp('visual_reinit_interval_s').value)
        self.image_buffer_s = float(gp('image_buffer_s').value)

        hfov = float(gp('camera_hfov_rad').value)
        width = int(gp('image_width').value)
        self.fx = (width / 2.0) / math.tan(hfov / 2.0)

        self.tracker_name, self.tracker_factory = (None, None)
        if bool(gp('visual_tracking').value):
            self.tracker_name, self.tracker_factory = find_visual_tracker_factory()

        self.tracks = []
        self.next_key = 0
        self.next_id = 1
        self.latest_image = None
        self.last_count = 0
        self.yaw_history = deque(maxlen=600)        # (time_sec, yaw)
        self.image_buffer = deque()                 # (time_sec, Image) for the last few seconds
        self.pose_history = deque(maxlen=600)       # (time_sec, x, y, yaw) from odometry
        self.latest_scan = None
        self.person_memory = {}                     # id -> (x, y, last_time) in the odom frame
        self.image_w = width
        self.reid_radius = float(gp('reid_radius_m').value)
        self.reid_memory = float(gp('reid_memory_s').value)
        self.bearing_window = float(gp('bearing_window_rad').value)
        self.person_height = float(gp('person_height_m').value)
        self.merge_radius = float(gp('merge_radius_m').value)

        self.create_subscription(Detection2DArray, gp('detections_topic').value, self.detections_callback, 10)
        self.create_subscription(Image, gp('image_topic').value, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, gp('odom_topic').value, self.odom_callback, 50)
        self.create_subscription(LaserScan, gp('scan_topic').value, self.scan_callback, qos_profile_sensor_data)
        self.positions_pub = self.create_publisher(PoseArray, '/perception/person_positions', 10)
        self.locations_pub = self.create_publisher(Detection3DArray, '/perception/person_locations', 10)

        self.tracked_pub = self.create_publisher(Detection2DArray, '/perception/tracked_persons', 10)
        self.count_pub = self.create_publisher(Int32, '/perception/person_count', 10)
        self.image_pub = self.create_publisher(Image, '/perception/tracked_image', 10)

        self.create_timer(1.0 / float(gp('draw_rate_hz').value), self.draw_tracks)
        self.create_timer(0.2, self.publish_timer)   # steady 5 Hz output, whatever YOLO's speed

        visual = (f'{self.tracker_name} (YOLO must re-confirm every {self.max_visual_only:.0f} s)'
                  if self.tracker_factory else 'OFF (no OpenCV tracker available - install opencv-contrib-python)')
        self.get_logger().info(
            f'Person tracker ready: confirm after {self.min_hits} hits, '
            f'ego-motion compensation {"ON" if self.compensate else "OFF"} ({self.fx:.0f} px/rad), '
            f'visual tracking {visual}'
        )

    # ---------- robot heading ----------

    def odom_callback(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.yaw_history.append((t, yaw))
        self.pose_history.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

    def scan_callback(self, msg):
        self.latest_scan = msg

    def pose_at(self, t):
        if not self.pose_history:
            return None
        s = min(self.pose_history, key=lambda p: abs(p[0] - t))
        return s[1], s[2], s[3]

    def scan_ranges_at(self, bearing):
        """All valid LiDAR ranges within a small window around `bearing` (robot frame, + = left)."""
        scan = self.latest_scan
        if scan is None:
            return []
        out = []
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max) or math.isinf(r) or math.isnan(r):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(wrap_angle(angle - bearing)) <= self.bearing_window:
                out.append(r)
        return out

    def localize(self, box, stamp):
        """Person position (x, y) in the odom frame.
        Direction: from the box's horizontal position. Distance: the LiDAR reading in that direction that
        agrees with a size estimate (a person ~1.7 m tall who is h pixels high is ~fx*1.7/h metres away)."""
        pose = self.pose_at(stamp)
        if pose is None:
            return None
        cx = (box[0] + box[2]) / 2.0
        h = max(box[3] - box[1], 1.0)
        bearing = -math.atan2(cx - self.image_w / 2.0, self.fx)   # image right = robot right = negative angle
        size_dist = self.fx * self.person_height / h
        rng = size_dist
        candidates = self.scan_ranges_at(bearing)
        if candidates:
            closest = min(candidates, key=lambda r: abs(r - size_dist))
            if 0.75 * size_dist <= closest <= 1.35 * size_dist:
                rng = closest
        x, y, yaw = pose
        return (x + rng * math.cos(yaw + bearing), y + rng * math.sin(yaw + bearing))

    def frame_at(self, stamp):
        """Full-resolution BGR frame closest to `stamp` from the recent image buffer."""
        if not self.image_buffer:
            return None
        t, msg = min(self.image_buffer, key=lambda e: abs(e[0] - stamp))
        if abs(t - stamp) > 0.2:
            return None
        try:
            return image_msg_to_bgr(msg)
        except ValueError:
            return None

    def update_appearance(self, tr, stamp, box):
        frame = self.frame_at(stamp)
        if frame is None:
            return
        hist = appearance_hist(frame, box)
        if hist is None:
            return
        old = getattr(tr, 'hist', None)
        tr.hist = hist if old is None else cv2.addWeighted(old, 0.7, hist, 0.3, 0)

    def remember(self, tr, stamp, now):
        pos = self.localize(tr.box, stamp)
        if pos is not None:
            old = getattr(tr, 'pos', None)
            tr.pos = pos if old is None else (0.5 * pos[0] + 0.5 * old[0], 0.5 * pos[1] + 0.5 * old[1])
        if tr.id is not None:
            mem = self.person_memory.setdefault(tr.id, {})
            mem['t'] = now
            if getattr(tr, 'pos', None) is not None:
                mem['pos'] = tr.pos
            if getattr(tr, 'hist', None) is not None:
                mem['hist'] = tr.hist

    def reidentify(self, tr, stamp, now):
        """ID of a previously seen person matching this new track. Position and clothing colours are weighed
        together: the same spot needs only a loose colour match, a different spot needs a very strong one."""
        pos = self.localize(tr.box, stamp)
        tr.pos = pos
        hist = getattr(tr, 'hist', None)
        busy = {t.id for t in self.tracks if t.id is not None and t is not tr and now - t.last_seen < 1.0}
        best, best_score, notes = None, 0.0, []
        for pid, mem in self.person_memory.items():
            if pid in busy or now - mem.get('t', -1e9) > self.reid_memory:
                continue
            dist = None
            if pos is not None and mem.get('pos') is not None:
                dist = math.hypot(mem['pos'][0] - pos[0], mem['pos'][1] - pos[1])
            sim = None
            if hist is not None and mem.get('hist') is not None:
                sim = float(cv2.compareHist(hist, mem['hist'], cv2.HISTCMP_CORREL))
            near = None if dist is None else dist <= self.reid_radius

            if sim is None:
                ok, score = near is True, 0.5
            elif near is True:
                ok, score = sim >= 0.3, sim + 0.5      # same spot: accept unless the clothes clearly differ
            elif near is None:
                ok, score = sim >= 0.6, sim            # position unknown: rely on the clothes
            else:
                ok, score = sim >= 0.85, sim           # different spot: only a very strong clothes match

            d_txt = '?' if dist is None else f'{dist:.1f} m'
            s_txt = '?' if sim is None else f'{sim:.2f}'
            notes.append(f'#{pid}: dist {d_txt}, clothes {s_txt}')
            if ok and score > best_score:
                best, best_score = pid, score

        if best is None and notes:
            self.get_logger().info('No re-identification match (' + '; '.join(notes) + ')')
        if best is not None:
            for old in self.tracks:   # retire any stale track still carrying that ID
                if old is not tr and old.id == best:
                    old.id = None
                    old.visual = None
                    old.last_seen = old.last_alive = -1e9
        return best

    def yaw_at(self, t):
        if not self.yaw_history:
            return None
        return min(self.yaw_history, key=lambda s: abs(s[0] - t))[1]

    def latest_yaw(self):
        return self.yaw_history[-1][1] if self.yaw_history else None

    def shift_for_rotation(self, box, yaw_from, yaw_to):
        """Turning left (+yaw) moves everything in view to the right (+x)."""
        if not self.compensate or yaw_from is None or yaw_to is None:
            return box
        dx = self.fx * wrap_angle(yaw_to - yaw_from)
        return box + np.array([dx, 0.0, dx, 0.0])

    def predict_box(self, tr, yaw):
        return self.shift_for_rotation(tr.box, tr.yaw, yaw)

    # ---------- images and visual tracking ----------

    def small_bgr(self, msg):
        frame = image_msg_to_bgr(msg)
        if self.scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        return frame

    def from_small(self, bb):
        x, y, w, h = bb
        s = self.scale
        return np.array([x / s, y / s, (x + w) / s, (y + h) / s], dtype=float)

    def plausible(self, box, tr, img_w, img_h):
        w, h = box[2] - box[0], box[3] - box[1]
        if w < 8 or h < 16:
            return False
        inside = intersection(box, np.array([0.0, 0.0, img_w, img_h])) / max(box_area(box), 1e-6)
        if inside < 0.5:                       # mostly outside the image: the person has left the frame
            return False
        ratio = box_area(box) / max(tr.ref_area, 1e-6)
        return 0.3 <= ratio <= 3.0             # size must stay person-like

    def image_callback(self, msg):
        self.latest_image = msg
        t = stamp_to_sec(msg.header.stamp)
        self.image_buffer.append((t, msg))
        while self.image_buffer and self.image_buffer[0][0] < t - self.image_buffer_s:
            self.image_buffer.popleft()

        active = [tr for tr in self.tracks if tr.id is not None and tr.visual is not None]
        if not active:
            return
        frame = self.small_bgr(msg)
        yaw = self.yaw_at(t)
        now = time.monotonic()
        for tr in active:
            ok, bb = tr.visual.update(frame)
            box = self.from_small(bb) if ok else None
            if box is None or not self.plausible(box, tr, msg.width, msg.height):
                tr.visual = None
                self.get_logger().info(f'Person #{tr.id}: visual lock lost')
                continue
            tr.box = box
            tr.yaw = yaw
            tr.last_alive = now
            # position is NOT updated here: only YOLO-confirmed boxes move a person on the map

    def start_visual(self, tr, box, stamp, now):
        """Start a visual tracker on the frame YOLO actually looked at, then catch it up to the newest frame."""
        if self.tracker_factory is None or not self.image_buffer:
            return
        buf = list(self.image_buffer)
        idx = min(range(len(buf)), key=lambda i: abs(buf[i][0] - stamp))
        if abs(buf[idx][0] - stamp) > 0.2:
            # That exact frame is gone: start on the newest frame, with the box moved for any rotation since
            idx = len(buf) - 1
            box = self.shift_for_rotation(box, self.yaw_at(stamp), self.yaw_at(buf[idx][0]))

        s = self.scale
        first = self.small_bgr(buf[idx][1])
        fh, fw = first.shape[:2]
        x1, y1, x2, y2 = box * s
        # Trim the box to the image: OpenCV rejects boxes that stick out past the edge
        x1, y1 = min(max(0.0, x1), fw - 3.0), min(max(0.0, y1), fh - 3.0)
        x2, y2 = min(max(x1 + 2.0, x2), fw - 1.0), min(max(y1 + 2.0, y2), fh - 1.0)
        rect = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))
        try:
            vt = self.tracker_factory()
            vt.init(first, rect)
        except cv2.error as e:
            self.get_logger().warn(f'Could not start visual tracker: {e}')
            tr.visual = None
            return

        cur_box, cur_stamp = box.copy(), buf[idx][0]
        for t, m in buf[idx + 1:]:
            ok, bb = vt.update(self.small_bgr(m))
            if not ok:
                tr.visual = None
                return
            cur_box, cur_stamp = self.from_small(bb), t

        tr.visual = vt
        tr.box = cur_box
        tr.yaw = self.yaw_at(cur_stamp)
        tr.ref_area = box_area(box)
        tr.last_visual_init = now

    # ---------- YOLO detections ----------

    def suppress_duplicates(self, dets):
        dets = sorted(dets, key=lambda d: d[1], reverse=True)
        kept = []
        for box, conf in dets:
            duplicate = any(
                containment(box, k_box) >= self.containment_threshold or iou(box, k_box) >= 0.5
                for k_box, _ in kept
            )
            if not duplicate:
                kept.append((box, conf))
        return kept

    def detections_callback(self, msg):
        now = time.monotonic()
        det_stamp = stamp_to_sec(msg.header.stamp)
        yaw_det = self.yaw_at(det_stamp)

        dets = [
            (det_to_box(d), float(d.results[0].hypothesis.score) if d.results else 0.0)
            for d in msg.detections
        ]
        dets = self.suppress_duplicates(dets)

        predicted = [self.predict_box(tr, yaw_det) for tr in self.tracks]

        candidates = []
        for ti, pred in enumerate(predicted):
            for di, (box, _) in enumerate(dets):
                overlap = iou(pred, box)
                dist = center_distance(pred, box)
                if overlap >= self.iou_threshold or dist <= self.max_center_px:
                    candidates.append((overlap, -dist, ti, di))
        candidates.sort(reverse=True)

        used_tracks, used_dets = set(), set()
        for _, _, ti, di in candidates:
            if ti in used_tracks or di in used_dets:
                continue
            used_tracks.add(ti)
            used_dets.add(di)
            self.on_match(self.tracks[ti], predicted[ti], dets[di], yaw_det, det_stamp, now)

        for di, (box, conf) in enumerate(dets):
            if di not in used_dets:
                track = Track(self.next_key, box, conf, yaw_det, now)
                self.next_key += 1
                self.tracks.append(track)
                self.maybe_confirm(track, det_stamp, now)

        self.prune(now)

    def on_match(self, tr, predicted_box, det, yaw, det_stamp, now):
        box, conf = det
        a = self.alpha
        tr.conf = a * conf + (1.0 - a) * tr.conf
        tr.hits += 1
        tr.last_seen = now
        tr.last_alive = now

        if tr.id is not None and tr.visual is not None:
            # Visual lock active: keep its (newer) box, re-anchor it on YOLO now and then or if they disagree
            if now - tr.last_visual_init > self.reinit_interval or iou(predicted_box, box) < 0.5:
                self.start_visual(tr, box, det_stamp, now)
        else:
            tr.box = a * box + (1.0 - a) * predicted_box
            tr.yaw = yaw
            if tr.id is not None:
                self.start_visual(tr, box, det_stamp, now)

        self.update_appearance(tr, det_stamp, box)
        self.remember(tr, det_stamp, now)
        self.maybe_confirm(tr, det_stamp, now)

    def maybe_confirm(self, tr, det_stamp, now):
        if tr.id is not None or tr.hits < self.min_hits:
            return
        best, best_dist = None, self.relink_px
        for old in self.tracks:
            if old is tr or old.id is None or now - old.last_seen < 1.0:
                continue
            dist = center_distance(self.predict_box(old, tr.yaw), tr.box)
            if dist <= best_dist:
                best, best_dist = old, dist
        if best is not None:
            tr.id = best.id
            tr.first_seen = best.first_seen
            best.id = None
            best.visual = None
            best.last_seen = best.last_alive = -1e9
            self.get_logger().info(f'Person #{tr.id} re-acquired (confidence {tr.conf:.2f})')
        else:
            reid = self.reidentify(tr, det_stamp, now)
            if reid is not None:
                tr.id = reid
                self.get_logger().info(f'Person #{tr.id} re-identified at ({tr.pos[0]:.1f}, {tr.pos[1]:.1f})')
            else:
                tr.id = self.next_id
                self.next_id += 1
                where = f' at ({tr.pos[0]:.1f}, {tr.pos[1]:.1f})' if getattr(tr, 'pos', None) else ''
                self.get_logger().info(f'Person #{tr.id} appeared (confidence {tr.conf:.2f}){where}')
        self.remember(tr, det_stamp, now)
        self.start_visual(tr, tr.box, det_stamp, now)

    def prune(self, now):
        alive = []
        for tr in self.tracks:
            if tr.id is None:
                if now - tr.last_seen <= self.tentative_max_age:
                    alive.append(tr)
                continue
            if now - tr.last_alive <= self.max_age and now - tr.last_seen <= self.max_visual_only:
                alive.append(tr)
                continue
            reason = ('no YOLO re-confirmation for '
                      f'{now - tr.last_seen:.0f} s' if now - tr.last_seen > self.max_visual_only
                      else 'out of sight')
            self.get_logger().info(
                f'Person #{tr.id} lost ({reason}; tracked for {tr.last_alive - tr.first_seen:.1f} s)'
            )
            tr.visual = None
        self.tracks = alive
        self.merge_duplicates()

    def merge_duplicates(self):
        """Two confirmed tracks on the same person (same spot in the room, or overlapping boxes): keep the older ID."""
        confirmed = sorted((t for t in self.tracks if t.id is not None), key=lambda t: t.id)
        yaw = self.latest_yaw()
        removed = set()
        for i, a in enumerate(confirmed):
            if a.key in removed:
                continue
            for b in confirmed[i + 1:]:
                if b.key in removed:
                    continue
                pa, pb = getattr(a, 'pos', None), getattr(b, 'pos', None)
                same_place = pa is not None and pb is not None and math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < self.merge_radius
                same_box = iou(self.predict_box(a, yaw), self.predict_box(b, yaw)) > 0.4
                if not (same_place or same_box):
                    continue
                if b.last_seen > a.last_seen:   # b has the fresher view of this person: keep its box
                    a.box, a.yaw, a.visual = b.box, b.yaw, b.visual
                    a.last_visual_init, a.ref_area = b.last_visual_init, b.ref_area
                    if pb is not None:
                        a.pos = pb
                a.last_seen = max(a.last_seen, b.last_seen)
                a.last_alive = max(a.last_alive, b.last_alive)
                a.hits += b.hits
                removed.add(b.key)
                self.person_memory.pop(b.id, None)
                self.get_logger().info(f'Person #{b.id} merged into Person #{a.id} (same person)')
        if removed:
            self.tracks = [t for t in self.tracks if t.key not in removed]

    # ---------- outputs ----------

    def publish_tracks(self, header, yaw):
        confirmed = [t for t in self.tracks if t.id is not None]

        out = Detection2DArray()
        out.header = header
        for tr in confirmed:
            x1, y1, x2, y2 = self.predict_box(tr, yaw)
            d = Detection2D()
            d.header = header
            d.id = str(tr.id)
            d.bbox.center.position.x = float((x1 + x2) / 2.0)
            d.bbox.center.position.y = float((y1 + y2) / 2.0)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = float(tr.conf)
            d.results.append(hyp)
            out.detections.append(d)
        self.tracked_pub.publish(out)

        poses = PoseArray()
        poses.header.stamp = header.stamp
        poses.header.frame_id = 'odom'
        for tr in confirmed:
            pos = getattr(tr, 'pos', None)
            if pos is None:
                continue
            pose = Pose()
            pose.position.x, pose.position.y = float(pos[0]), float(pos[1])
            pose.orientation.w = 1.0
            poses.poses.append(pose)
        self.positions_pub.publish(poses)

        located = Detection3DArray()
        located.header.stamp = header.stamp
        located.header.frame_id = 'odom'
        for tr in confirmed:
            pos = getattr(tr, 'pos', None)
            if pos is None:
                continue
            d3 = Detection3D()
            d3.header = located.header
            d3.id = str(tr.id)
            d3.bbox.center.position.x = float(pos[0])
            d3.bbox.center.position.y = float(pos[1])
            d3.bbox.center.position.z = 0.85
            d3.bbox.center.orientation.w = 1.0
            d3.bbox.size.x, d3.bbox.size.y, d3.bbox.size.z = 0.4, 0.4, 1.7
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = float(tr.conf)
            d3.results.append(hyp)
            located.detections.append(d3)
        self.locations_pub.publish(located)

        count = len(confirmed)
        self.count_pub.publish(Int32(data=count))
        if count != self.last_count:
            self.get_logger().info(f'People present: {count}')
            self.last_count = count

    def publish_timer(self):
        """Publish tracked people at a steady rate, independent of how fast YOLO runs."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_optical_link'
        self.publish_tracks(header, self.latest_yaw())

    def draw_tracks(self):
        if self.latest_image is None:
            return
        try:
            frame = image_msg_to_bgr(self.latest_image)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        now = time.monotonic()
        yaw_now = self.latest_yaw()
        for tr in self.tracks:
            if tr.id is None:
                continue
            since_yolo = now - tr.last_seen
            since_alive = now - tr.last_alive
            if since_yolo < 0.6:
                color, suffix = (0, 255, 0), ''                         # green: YOLO sees them
            elif tr.visual is not None and since_alive < 0.5:
                color, suffix = (255, 255, 0), ' [visual]'              # cyan: held by the visual tracker
            else:
                color, suffix = (0, 165, 255), f' (lost {since_alive:.1f}s)'   # orange: coasting
            x1, y1, x2, y2 = (int(v) for v in self.predict_box(tr, yaw_now))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f'Person #{tr.id} {tr.conf:.2f}{suffix}', (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.image_pub.publish(bgr_to_image_msg(frame, self.latest_image.header))


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
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
