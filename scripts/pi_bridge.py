#!/usr/bin/env python3
"""Bridge between the robot's camera, the pi0.5 policy server on RSL, and the
operator console.

Subscribes to the compressed color feed, and on request (or continuously)
sends the latest frame + prompt to the openpi websocket policy server, then
publishes the returned action chunk as JSON for the web UI.

Topics:
  /pi/request  std_msgs/String  JSON {"prompt": str, "mode": "once"|"auto"|"stop",
                                      "rate_hz": float (auto mode, default 0.5)}
  /pi/result   std_msgs/String  JSON {ok, prompt, latency_ms, horizon, dims,
                                      actions, error, server}

Run inside the container (needs: pip install msgpack websockets):
  python3 scripts/pi_bridge.py --ros-args -p server_host:=RSL
"""
import functools
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    import msgpack
    import websockets.sync.client
except ImportError as e:
    raise SystemExit(f'missing dep ({e.name}); run: pip3 install msgpack websockets')


# ── vendored from openpi-client (msgpack_numpy.py) ───────────────────────────
def _pack_array(obj):
    if isinstance(obj, np.ndarray):
        return {b'__ndarray__': True, b'data': obj.tobytes(),
                b'dtype': obj.dtype.str, b'shape': obj.shape}
    if isinstance(obj, np.generic):
        return {b'__npgeneric__': True, b'data': obj.item(), b'dtype': obj.dtype.str}
    return obj


def _unpack_array(obj):
    if b'__ndarray__' in obj:
        return np.ndarray(buffer=obj[b'data'], dtype=np.dtype(obj[b'dtype']),
                          shape=obj[b'shape'])
    if b'__npgeneric__' in obj:
        return np.dtype(obj[b'dtype']).type(obj[b'data'])
    return obj


packb = functools.partial(msgpack.packb, default=_pack_array)
unpackb = functools.partial(msgpack.unpackb, object_hook=_unpack_array)


def resize_with_pad(img, h, w):
    """Letterbox-resize uint8 HWC image to (h, w) preserving aspect."""
    import cv2
    ih, iw = img.shape[:2]
    scale = min(h / ih, w / iw)
    nh, nw = int(ih * scale), int(iw * scale)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    out = np.zeros((h, w, 3), dtype=np.uint8)
    top, left = (h - nh) // 2, (w - nw) // 2
    out[top:top + nh, left:left + nw] = resized
    return out


class PiBridge(Node):
    def __init__(self):
        super().__init__('pi_bridge')
        self.declare_parameter('server_host', 'RSL')
        self.declare_parameter('server_port', 8000)
        self.declare_parameter('camera_topic', '/camera/color/image_raw/compressed')
        self.host = self.get_parameter('server_host').value
        self.port = self.get_parameter('server_port').value

        self.latest_jpeg = None
        self.prompt = 'pick up the sock'
        self.auto = False
        self.rate_hz = 0.5
        self.ws = None
        self.lock = threading.Lock()
        self.wake = threading.Event()

        self.create_subscription(CompressedImage,
                                 self.get_parameter('camera_topic').value,
                                 self.on_image, 5)
        self.create_subscription(String, '/pi/request', self.on_request, 5)
        self.result_pub = self.create_publisher(String, '/pi/result', 5)

        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f'pi_bridge up; policy server ws://{self.host}:{self.port}')

    def on_image(self, msg):
        self.latest_jpeg = bytes(msg.data)

    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            self.publish_error('bad request JSON')
            return
        self.prompt = req.get('prompt', self.prompt) or self.prompt
        mode = req.get('mode', 'once')
        self.rate_hz = float(req.get('rate_hz', self.rate_hz))
        if mode == 'auto':
            self.auto = True
        elif mode == 'stop':
            self.auto = False
        self.wake.set()

    # ── policy server client ────────────────────────────────────────────────
    def connect(self):
        if self.ws is not None:
            return True
        try:
            self.ws = websockets.sync.client.connect(
                f'ws://{self.host}:{self.port}', compression=None, max_size=None,
                open_timeout=5)
            unpackb(self.ws.recv())  # server metadata
            return True
        except Exception as e:
            self.ws = None
            self.publish_error(f'server unreachable: {e}')
            return False

    def infer_once(self):
        if self.latest_jpeg is None:
            self.publish_error('no camera frame yet')
            return
        if not self.connect():
            return
        import cv2
        img = cv2.imdecode(np.frombuffer(self.latest_jpeg, np.uint8),
                           cv2.IMREAD_COLOR)[:, :, ::-1]  # BGR→RGB
        obs = {
            'observation/exterior_image_1_left': resize_with_pad(img, 224, 224),
            'observation/wrist_image_left': np.zeros((224, 224, 3), np.uint8),
            'observation/joint_position': np.zeros(7),
            'observation/gripper_position': np.zeros(1),
            'prompt': self.prompt,
        }
        t0 = time.time()
        try:
            self.ws.send(packb(obs))
            resp = self.ws.recv()
            if isinstance(resp, str):
                raise RuntimeError(resp[:200])
            actions = np.asarray(unpackb(resp)['actions'])
        except Exception as e:
            self.ws = None  # force reconnect next time
            self.publish_error(f'inference failed: {e}')
            return
        self.result_pub.publish(String(data=json.dumps({
            'ok': True,
            'prompt': self.prompt,
            'latency_ms': round(1000 * (time.time() - t0), 1),
            'horizon': int(actions.shape[0]),
            'dims': int(actions.shape[1]),
            'actions': np.round(actions, 4).tolist(),
            'server': f'{self.host}:{self.port}',
            'ts': time.time(),
        })))

    def publish_error(self, err):
        self.get_logger().warn(err)
        self.result_pub.publish(String(data=json.dumps(
            {'ok': False, 'error': err, 'prompt': self.prompt,
             'server': f'{self.host}:{self.port}', 'ts': time.time()})))

    def worker(self):
        while True:
            triggered = self.wake.wait(timeout=0.2)
            if triggered:
                self.wake.clear()
                with self.lock:
                    self.infer_once()
            if self.auto:
                with self.lock:
                    self.infer_once()
                time.sleep(max(0.2, 1.0 / self.rate_hz))


def main():
    rclpy.init()
    node = PiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
