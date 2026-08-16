#!/usr/bin/env python3
"""Bridge between the robot's camera/arm, an openpi policy server, and the
operator console.

Two policy styles:
  roarm (default) — our fine-tuned pi0 (see openpi_roarm/): sends the REAL
      observation (camera frame + current cartesian arm state [x,y,z,grip]
      from /teleop/state) and receives (horizon, 4) absolute cartesian
      setpoints. In "execute" mode it streams them to the arm as `goto:`
      actions on /teleop/action — the same channel the teleop console and
      sock_cycle use, so teleop_node stays the single serial owner and the
      browser E-STOP halts policy execution too.
  droid — legacy smoke-test against the stock pi0.5-DROID server (zeroed
      proprioception, 8-dim Franka actions). Inference only; never executed.

Topics:
  /pi/request  std_msgs/String JSON {"prompt": str,
                                     "mode": "once"|"auto"|"execute"|"stop",
                                     "rate_hz": float}   (auto-mode infer rate)
  /pi/result   std_msgs/String JSON {ok, prompt, latency_ms, horizon, dims,
                                     actions, executing, error, server}

The policy server usually runs on the GPU box with only SSH exposed; tunnel it:
  ssh -i ~/.ssh/runpod_ed25519 -p <pod-port> -N -L 8000:localhost:8000 root@<pod-ip>
then run with server_host:=localhost.

Run inside the container (needs: pip install msgpack websockets):
  python3 scripts/pi_bridge.py --ros-args -p server_host:=localhost
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
        self.declare_parameter('server_host', 'localhost')
        self.declare_parameter('server_port', 8000)
        self.declare_parameter('camera_topic', '/camera/color/image_raw/compressed')
        # Wrist camera for dual-view checkpoints (pi0_roarm_sock_cartesian_
        # wrist_lora). Empty = head-only, matching the v1 checkpoint; a
        # wrist-trained policy served WITHOUT this gets a masked-off wrist at
        # rollout -- a silent train/serve mismatch, so set it when serving v2+.
        self.declare_parameter('wrist_topic', '')
        self.declare_parameter('policy_style', 'roarm')   # 'roarm' | 'droid'
        self.declare_parameter('exec_rate_hz', 15.0)      # demo recording rate
        self.declare_parameter('exec_steps', 25)          # chunk prefix to run
                                                          # before re-planning
        self.declare_parameter('state_max_age_s', 1.0)
        self.declare_parameter('max_chunks', 12)  # execute auto-disarms after
                                                  # this many chunks per arm
        self.host = self.get_parameter('server_host').value
        self.port = self.get_parameter('server_port').value
        self.style = self.get_parameter('policy_style').value

        self.latest_jpeg = None
        self.latest_wrist_jpeg = None
        self.teleop = None          # last /teleop/state dict
        self.teleop_t = 0.0
        self.prompt = 'pick up the sock'
        self.mode = 'idle'          # idle | auto | execute
        self.chunks_run = 0
        self.rate_hz = 0.5
        self.ws = None
        self.lock = threading.Lock()
        self.wake = threading.Event()

        self.create_subscription(CompressedImage,
                                 self.get_parameter('camera_topic').value,
                                 self.on_image, 5)
        self.wrist_topic = self.get_parameter('wrist_topic').value
        if self.wrist_topic:
            self.create_subscription(CompressedImage, self.wrist_topic,
                                     self.on_wrist_image, 5)
        self.create_subscription(String, '/teleop/state', self.on_teleop, 5)
        self.create_subscription(String, '/pi/request', self.on_request, 5)
        self.result_pub = self.create_publisher(String, '/pi/result', 5)
        self.action_pub = self.create_publisher(String, '/teleop/action', 20)

        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(
            f'pi_bridge up ({self.style}); server ws://{self.host}:{self.port}')

    def on_image(self, msg):
        self.latest_jpeg = bytes(msg.data)

    def on_wrist_image(self, msg):
        self.latest_wrist_jpeg = bytes(msg.data)

    def on_teleop(self, msg):
        try:
            self.teleop = json.loads(msg.data)
            self.teleop_t = time.time()
        except ValueError:
            pass

    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            self.publish_error('bad request JSON')
            return
        self.prompt = req.get('prompt', self.prompt) or self.prompt
        mode = req.get('mode', 'once')
        self.rate_hz = float(req.get('rate_hz', self.rate_hz))
        if mode in ('auto', 'execute'):
            self.mode = mode
            self.chunks_run = 0
        elif mode == 'stop':
            self.mode = 'idle'
        # Every request and resulting mode is logged: a stop MUST be verifiable
        # from the log and from /pi/result's 'mode' field, never assumed from
        # the sender's side. (2026-07-24: an unverified stop left execute mode
        # silently armed and the policy picked up the sock on its own.)
        self.get_logger().info(f'request: {mode} -> mode={self.mode} prompt="{self.prompt}"')
        self.wake.set()

    # ── robot state ─────────────────────────────────────────────────────────
    def arm_state(self):
        """Current cartesian arm state [x, y, z, grip], or None if stale/absent.
        This is the REAL proprioception the fine-tuned policy was trained on —
        never fabricate it; refuse to infer without it."""
        if self.teleop is None:
            return None
        if time.time() - self.teleop_t > self.get_parameter('state_max_age_s').value:
            return None
        a = self.teleop.get('arm') or {}
        vals = [a.get('x'), a.get('y'), a.get('z'), a.get('t')]
        if any(v is None for v in vals):
            return None
        return np.asarray(vals, dtype=np.float32)

    def estopped(self):
        return bool((self.teleop or {}).get('estop'))

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

    def build_obs(self, img):
        if self.style == 'droid':
            return {
                'observation/exterior_image_1_left': resize_with_pad(img, 224, 224),
                'observation/wrist_image_left': np.zeros((224, 224, 3), np.uint8),
                'observation/joint_position': np.zeros(7),
                'observation/gripper_position': np.zeros(1),
                'prompt': self.prompt,
            }
        state = self.arm_state()
        if state is None:
            return None
        obs = {
            'observation/image': resize_with_pad(img, 224, 224),
            'observation/state': state,
            'prompt': self.prompt,
        }
        if self.wrist_topic:
            # a wrist-trained policy must never silently get a stale/absent
            # wrist view: refuse instead (same doctrine as arm_state)
            if self.latest_wrist_jpeg is None:
                return 'no-wrist'
            import cv2
            wimg = cv2.imdecode(
                np.frombuffer(self.latest_wrist_jpeg, np.uint8),
                cv2.IMREAD_COLOR)[:, :, ::-1]           # BGR -> RGB
            obs['observation/wrist_image'] = resize_with_pad(wimg, 224, 224)
        return obs

    def infer_once(self):
        """One inference. Returns the (horizon, dims) action array or None."""
        if self.latest_jpeg is None:
            self.publish_error('no camera frame yet')
            return None
        if not self.connect():
            return None
        import cv2
        img = cv2.imdecode(np.frombuffer(self.latest_jpeg, np.uint8),
                           cv2.IMREAD_COLOR)[:, :, ::-1]  # BGR→RGB
        obs = self.build_obs(img)
        if obs is None:
            self.publish_error('no fresh /teleop/state — is teleop_node up?')
            return None
        if obs == 'no-wrist':
            self.publish_error(f'no wrist frame on {self.wrist_topic} — '
                               'refusing to infer with a masked wrist view')
            return None
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
            return None
        self.result_pub.publish(String(data=json.dumps({
            'ok': True,
            'prompt': self.prompt,
            'latency_ms': round(1000 * (time.time() - t0), 1),
            'horizon': int(actions.shape[0]),
            'dims': int(actions.shape[1]),
            'actions': np.round(actions, 4).tolist(),
            'executing': self.mode == 'execute',
            'mode': self.mode,
            'server': f'{self.host}:{self.port}',
            'ts': time.time(),
        })))
        return actions

    def execute_chunk(self, actions):
        """Stream the first exec_steps actions to the arm as goto: setpoints.

        Every safety property is inherited from teleop_node's goto handler:
        E-STOP gating, workspace envelope clamp, single serial owner. This
        method additionally stops on estop/stale-state and returns False so
        the caller drops back to idle.
        """
        if self.style != 'roarm' or actions.shape[1] != 4:
            self.publish_error(f'refusing to execute {actions.shape} actions '
                               f'with style={self.style}')
            return False
        rate = self.get_parameter('exec_rate_hz').value
        steps = min(int(self.get_parameter('exec_steps').value), len(actions))
        for i in range(steps):
            if self.mode != 'execute' or self.estopped() or self.arm_state() is None:
                return False
            x, y, z, t = (float(v) for v in actions[i])
            self.action_pub.publish(String(data=f'goto:{x:.1f},{y:.1f},{z:.1f},{t:.3f}'))
            time.sleep(1.0 / rate)
        return True

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
                if self.mode not in ('auto', 'execute'):
                    with self.lock:
                        self.infer_once()
            if self.mode == 'auto':
                with self.lock:
                    self.infer_once()
                time.sleep(max(0.2, 1.0 / self.rate_hz))
            elif self.mode == 'execute':
                if self.chunks_run >= int(self.get_parameter('max_chunks').value):
                    self.get_logger().warn(
                        f'execute auto-disarmed after {self.chunks_run} chunks '
                        '(re-arm with another execute request)')
                    self.mode = 'idle'
                    continue
                with self.lock:
                    actions = self.infer_once()
                    if actions is None or not self.execute_chunk(actions):
                        self.get_logger().info('execute -> idle (failure/stop)')
                        self.mode = 'idle'   # any failure -> stop, loudly
                    else:
                        self.chunks_run += 1
                        self.get_logger().info(f'chunk {self.chunks_run} done')
                # receding horizon: immediately re-infer from the new state


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
