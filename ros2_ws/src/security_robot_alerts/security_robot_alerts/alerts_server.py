"""Security dashboard backend.

Listens to the security robot's ROS topics, stores restricted-zone alerts (grouped into incidents, with a
camera snapshot) in SQLite, and serves a web dashboard + JSON API with FastAPI."""
import json
import math
import os
import sqlite3
import threading
import time
from pathlib import Path

import cv2
import rclpy
import uvicorn
import yaml
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, Response, StreamingResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from vision_msgs.msg import Detection3DArray

from security_robot_perception.image_utils import image_msg_to_bgr

DATA_DIR = Path.home() / 'security-robot' / 'data'
SNAP_DIR = DATA_DIR / 'snapshots'
DB_PATH = DATA_DIR / 'security.db'
INCIDENT_WINDOW_S = 120.0     # re-sightings of the same person in the same zone within this = same alert


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------- storage ----------------

class AlertStore:
    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.lock = threading.Lock()
        with self.lock:
            self.db.execute('''CREATE TABLE IF NOT EXISTS alerts (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                started REAL, last_seen REAL,
                person_id TEXT, zone TEXT, level TEXT,
                x REAL, y REAL,
                sightings INTEGER DEFAULT 1,
                status TEXT DEFAULT 'in zone',
                snapshot TEXT,
                acknowledged INTEGER DEFAULT 0)''')
            self.db.commit()

    def open_incident(self, zone, person_id, now):
        with self.lock:
            row = self.db.execute(
                'SELECT * FROM alerts WHERE zone=? AND person_id=? AND acknowledged=0 AND last_seen>=? '
                'ORDER BY id DESC LIMIT 1', (zone, person_id, now - INCIDENT_WINDOW_S)).fetchone()
        return dict(row) if row else None

    def add(self, now, person_id, zone, level, x, y, snapshot):
        with self.lock:
            cur = self.db.execute(
                'INSERT INTO alerts (started, last_seen, person_id, zone, level, x, y, snapshot) '
                'VALUES (?, ?, ?, ?, ?, ?, ?, ?)', (now, now, person_id, zone, level, x, y, snapshot))
            self.db.commit()
            return cur.lastrowid

    def seen_again(self, alert_id, now, x, y):
        with self.lock:
            self.db.execute(
                "UPDATE alerts SET last_seen=?, x=?, y=?, sightings=sightings+1, status='in zone' WHERE id=?",
                (now, x, y, alert_id))
            self.db.commit()

    def set_status(self, zone, person_id, status):
        with self.lock:
            self.db.execute(
                'UPDATE alerts SET status=? WHERE id=(SELECT id FROM alerts WHERE zone=? AND person_id=? '
                'ORDER BY id DESC LIMIT 1)', (status, zone, person_id))
            self.db.commit()

    def recent(self, limit):
        with self.lock:
            rows = self.db.execute('SELECT * FROM alerts ORDER BY id DESC LIMIT ?', (limit,)).fetchall()
        return [dict(r) for r in rows]

    def acknowledge(self, alert_id):
        with self.lock:
            self.db.execute('UPDATE alerts SET acknowledged=1 WHERE id=?', (alert_id,))
            self.db.commit()


# ---------------- ROS side ----------------

class SecurityBridge(Node):
    def __init__(self, store):
        super().__init__('security_dashboard')
        self.store = store
        self.lock = threading.Lock()

        self.state = {'state': 'UNKNOWN', 'detail': 'waiting for the security engine'}
        self.state_time = None
        self.robot = None
        self.people = {}            # id -> (x, y, time)
        self.people_count = 0
        self.latest_frame = None
        self.latest_jpeg = None
        self.last_encode = 0.0

        self.create_subscription(String, '/security/zone_events', self.on_zone_event, 50)
        self.create_subscription(String, '/security/state', self.on_state, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(Detection3DArray, '/perception/person_locations', self.on_people, 10)
        self.create_subscription(Int32, '/perception/person_count', self.on_count, 10)
        self.create_subscription(Image, '/perception/tracked_image', self.on_image, qos_profile_sensor_data)

    # --- inputs ---

    def on_state(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.state = data
            self.state_time = time.time()

    def on_pose(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.robot = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw_from_quaternion(p.orientation)}

    def on_people(self, msg):
        now = time.time()
        with self.lock:
            for det in msg.detections:
                self.people[det.id] = (det.bbox.center.position.x, det.bbox.center.position.y, now)

    def on_count(self, msg):
        with self.lock:
            self.people_count = msg.data

    def on_image(self, msg):
        now = time.time()
        if now - self.last_encode < 0.2:          # encode at most 5 frames/s
            return
        self.last_encode = now
        try:
            frame = image_msg_to_bgr(msg)
        except ValueError:
            return
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.latest_frame = frame
                self.latest_jpeg = buf.tobytes()

    def on_zone_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event, zone, level = ev.get('event'), ev.get('zone'), ev.get('level')
        pid = str(ev.get('person_id'))
        x, y = float(ev.get('x', 0.0)), float(ev.get('y', 0.0))
        now = time.time()

        if event == 'zone_entered' and level == 'restricted':
            incident = self.store.open_incident(zone, pid, now)
            if incident:
                self.store.seen_again(incident['id'], now, x, y)
            else:
                snapshot = self.save_snapshot(pid, zone, now)
                alert_id = self.store.add(now, pid, zone, level, x, y, snapshot)
                self.get_logger().warn(f"ALERT #{alert_id}: Person #{pid} in restricted zone '{zone}'"
                                       f"{' - snapshot saved' if snapshot else ''}")
        elif event == 'zone_exited':
            self.store.set_status(zone, pid, 'left the zone')
        elif event == 'lost_sight':
            self.store.set_status(zone, pid, 'out of view')

    def save_snapshot(self, pid, zone, now):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return None
        stamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now))
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 28), (0, 0, 180), -1)
        cv2.putText(frame, f'INTRUDER  Person #{pid}  {zone}  {stamp}', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        name = f"{time.strftime('%Y%m%d-%H%M%S', time.localtime(now))}_person{pid}_{zone}.jpg"
        cv2.imwrite(str(SNAP_DIR / name), frame)
        return name

    # --- outputs for the API ---

    def status(self):
        now = time.time()
        with self.lock:
            people = [{'id': pid, 'x': x, 'y': y} for pid, (x, y, t) in self.people.items() if now - t < 3.0]
            return {
                'state': self.state,
                'state_age_s': None if self.state_time is None else round(now - self.state_time, 1),
                'robot': self.robot,
                'people': people,
                'people_count': self.people_count,
                'camera_live': self.latest_jpeg is not None,
                'time': now,
            }

    def jpeg(self):
        with self.lock:
            return self.latest_jpeg


# ---------------- map + zones ----------------

def load_map():
    try:
        yml = Path(get_package_share_directory('security_robot_webots')) / 'maps' / 'room_map.yaml'
        meta = yaml.safe_load(yml.read_text())
        img_path = Path(meta['image'])
        if not img_path.is_absolute():
            img_path = yml.parent / img_path
        img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        ok, png = cv2.imencode('.png', img)
        return png.tobytes(), {
            'resolution': float(meta['resolution']),
            'origin': [float(meta['origin'][0]), float(meta['origin'][1])],
            'width': int(img.shape[1]),
            'height': int(img.shape[0]),
        }
    except Exception as e:
        print(f'[dashboard] map not loaded: {e}')
        return None, None


def load_zones():
    try:
        path = Path(get_package_share_directory('security_robot_security')) / 'config' / 'zones.yaml'
        data = yaml.safe_load(path.read_text())
        return [{'name': z['name'], 'level': z.get('level', 'restricted'), 'polygon': z['polygon']}
                for z in data.get('zones', [])]
    except Exception as e:
        print(f'[dashboard] zones not loaded: {e}')
        return []


# ---------------- web side ----------------

def make_app(node, store, map_png, map_meta, zones):
    app = FastAPI(title='Security robot dashboard')
    index = Path(get_package_share_directory('security_robot_alerts')) / 'web' / 'index.html'

    @app.get('/', response_class=HTMLResponse)
    def home():
        return index.read_text()

    @app.get('/api/status')
    def status():
        return node.status()

    @app.get('/api/alerts')
    def alerts(limit: int = 50):
        return store.recent(min(max(limit, 1), 500))

    @app.post('/api/alerts/{alert_id}/ack')
    def acknowledge(alert_id: int):
        store.acknowledge(alert_id)
        return {'ok': True}

    @app.get('/api/snapshots/{name}')
    def snapshot(name: str):
        path = (SNAP_DIR / name).resolve()
        if path.parent != SNAP_DIR.resolve() or not path.exists():
            raise HTTPException(status_code=404)
        return FileResponse(path, media_type='image/jpeg')

    @app.get('/api/camera.mjpg')
    def camera():
        def frames():
            while True:
                jpg = node.jpeg()
                if jpg:
                    yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n'
                time.sleep(0.2)
        return StreamingResponse(frames(), media_type='multipart/x-mixed-replace; boundary=frame')

    @app.get('/api/map')
    def map_info():
        return {'map': map_meta, 'zones': zones}

    @app.get('/api/map.png')
    def map_image():
        if map_png is None:
            raise HTTPException(status_code=404)
        return Response(map_png, media_type='image/png')

    return app


def main():
    SNAP_DIR.mkdir(parents=True, exist_ok=True)
    store = AlertStore(DB_PATH)

    rclpy.init()
    node = SecurityBridge(store)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    map_png, map_meta = load_map()
    zones = load_zones()
    port = int(os.environ.get('DASHBOARD_PORT', '8080'))
    node.get_logger().info(f'Dashboard running - open http://localhost:{port} in your browser')
    try:
        uvicorn.run(make_app(node, store, map_png, map_meta, zones), host='0.0.0.0', port=port, log_level='warning')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
