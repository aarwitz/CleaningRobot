#!/usr/bin/env python3
"""Emitter-frame splitter for D455 emitter_on_off alternation.

With `depth_module.emitter_on_off:=true` the RealSense alternates the IR dot
projector every frame. The dots are essential for stereo depth on textureless
surfaces but poison cuVSLAM's feature tracking (projected dots move WITH the
robot, so VO reads ~zero motion — the root cause of every "cuVSLAM is
untrustworthy" finding on this robot, diagnosed 2026-07-02).

Splits streams by the per-frame `frame_emitter_mode` metadata:
  emitter OFF frames -> /camera/infra1_off/image_raw + /camera/infra2_off/image_raw
                        published as an ATOMIC STEREO PAIR (same stamp, both or
                        neither) — cuVSLAM needs matching stamps; per-stream
                        independent gating collapsed the pair rate to ~5 Hz.
  emitter ON  frames -> /camera/depth_on/image_raw (for nvblox/perception)

NVIDIA ships realsense_splitter (C++) for exactly this, but in this container
its message_filters ExactTime sync never fires for the infra pair despite
bit-exact stamps (verified empirically); this is the same logic in plain rclpy.
The emitter state comes from infra1's metadata only — both imagers share the
same sensor clock and emitter phase.
"""

import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import Metadata


def _trim(d, n=60):
    while len(d) > n:
        d.pop(next(iter(d)))


class EmitterSplitter(Node):
    def __init__(self):
        super().__init__('emitter_splitter')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=30)
        self._lock = threading.Lock()
        # stereo pairing: stamp -> {'i1':, 'i2':, 'mode':}
        self._pairs = {}
        self._pair_count = 0
        # depth pairing: stamp -> img / mode
        self._d_imgs, self._d_modes = {}, {}
        self._depth_count = 0

        self.pub_i1 = self.create_publisher(Image, '/camera/infra1_off/image_raw', qos)
        self.pub_i2 = self.create_publisher(Image, '/camera/infra2_off/image_raw', qos)
        self.pub_d = self.create_publisher(Image, '/camera/depth_on/image_raw', qos)

        self.create_subscription(Image, '/camera/infra1/image_rect_raw',
                                 lambda m: self._on_pair_part(m, 'i1'), qos)
        self.create_subscription(Image, '/camera/infra2/image_rect_raw',
                                 lambda m: self._on_pair_part(m, 'i2'), qos)
        self.create_subscription(Metadata, '/camera/infra1/metadata',
                                 self._on_infra_meta, qos)
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 self._on_depth, qos)
        self.create_subscription(Metadata, '/camera/depth/metadata',
                                 self._on_depth_meta, qos)
        self.create_timer(10.0, self._report)

        # --- brightness governor -------------------------------------------
        # Manual exposure/gain are required for dot-free off frames (AE runs
        # the window to ~32ms and the off frame integrates the on-phase; see
        # launch file). Fixed settings can't follow room-to-room lighting, so
        # this loop re-implements a slow AE over the OFF frames only: keep the
        # off-frame mean inside [bright_lo, bright_hi] by scaling gain (then
        # exposure, never above exposure_max to preserve dot-free frames).
        # exposure/gain writes are LOCKED while emitter_on_off is active, so
        # every adjustment does the unlock dance: on_off false -> set -> true.
        self.declare_parameter('governor_enabled', True)
        self.declare_parameter('bright_lo', 30.0)
        self.declare_parameter('bright_hi', 110.0)
        self.declare_parameter('bright_target', 60.0)
        self.declare_parameter('exposure_max', 14000)   # us; bleed-free ceiling @30fps
        self.declare_parameter('exposure_min', 2000)
        self._gov_enabled = self.get_parameter('governor_enabled').value
        self._bright_lo = self.get_parameter('bright_lo').value
        self._bright_hi = self.get_parameter('bright_hi').value
        self._bright_target = self.get_parameter('bright_target').value
        self._exp_max = self.get_parameter('exposure_max').value
        self._exp_min = self.get_parameter('exposure_min').value
        self._ema = None            # EMA of off-frame mean brightness
        self._cur_gain = 220.0      # mirrors launch boot values; resynced on set
        self._cur_exp = float(self._exp_max)
        self._gov_busy = False
        self._gov_cooldown_until = 0.0
        self._param_cli = self.create_client(SetParameters,
                                             '/camera/camera/set_parameters')
        if self._gov_enabled:
            self.create_timer(5.0, self._governor_tick)
        self.get_logger().info('emitter splitter up (stereo-paired OFF frames, ON depth)')

    @staticmethod
    def _key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    @staticmethod
    def _mode_of(meta_msg):
        try:
            return int(json.loads(meta_msg.json_data).get('frame_emitter_mode', -1))
        except (ValueError, TypeError):
            return -1

    # --- stereo pair path ---------------------------------------------------

    def _on_pair_part(self, msg, slot):
        k = self._key(msg)
        with self._lock:
            e = self._pairs.setdefault(k, {})
            e[slot] = msg
            done = 'i1' in e and 'i2' in e and 'mode' in e
            if done:
                self._pairs.pop(k)
            else:
                _trim(self._pairs)
        if done:
            self._emit_pair(e)

    def _on_infra_meta(self, msg):
        k = self._key(msg)
        mode = self._mode_of(msg)
        with self._lock:
            e = self._pairs.setdefault(k, {})
            e['mode'] = mode
            done = 'i1' in e and 'i2' in e
            if done:
                self._pairs.pop(k)
            else:
                _trim(self._pairs)
        if done:
            self._emit_pair(e)

    def _emit_pair(self, e):
        if e['mode'] == 0:
            self.pub_i1.publish(e['i1'])
            self.pub_i2.publish(e['i2'])
            self._pair_count += 1
            if self._pair_count % 15 == 1:  # ~1 Hz brightness sample
                self._sample_brightness(e['i1'])

    # --- brightness governor --------------------------------------------------

    def _sample_brightness(self, msg):
        mv = memoryview(msg.data)[::631]
        mean = sum(mv) / max(1, len(mv))
        self._ema = mean if self._ema is None else 0.7 * self._ema + 0.3 * mean

    def _governor_tick(self):
        if (self._gov_busy or self._ema is None
                or time.monotonic() < self._gov_cooldown_until):
            return
        if self._bright_lo <= self._ema <= self._bright_hi:
            return
        scale = self._bright_target / max(1.0, self._ema)
        # gain first (16..248), spill remainder into exposure (2000..exp_max)
        want = self._cur_gain * scale
        new_gain = min(248.0, max(16.0, want))
        resid = want / new_gain
        new_exp = min(float(self._exp_max),
                      max(float(self._exp_min), self._cur_exp * resid))
        if (abs(new_gain - self._cur_gain) < 8.0
                and abs(new_exp - self._cur_exp) < 500.0):
            return  # saturated against limits; nothing meaningful to change
        self._gov_busy = True
        threading.Thread(target=self._apply_exposure,
                         args=(int(new_exp), int(new_gain)), daemon=True).start()

    def _set_cam(self, name, ptype, value):
        p = Parameter(name=name)
        if ptype == 'bool':
            p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                     bool_value=value)
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                     integer_value=value)
        res = self._param_cli.call(SetParameters.Request(parameters=[p]))
        return res is not None and all(r.successful for r in res.results)

    def _apply_exposure(self, exp, gain):
        """Worker thread: unlock dance around the locked exposure/gain writes."""
        try:
            if not self._param_cli.wait_for_service(timeout_sec=2.0):
                return
            ok = (self._set_cam('depth_module.emitter_on_off', 'bool', False)
                  and self._set_cam('depth_module.exposure', 'int', exp)
                  and self._set_cam('depth_module.gain', 'int', gain))
            # ALWAYS restore alternation, even if the sets failed midway.
            restored = self._set_cam('depth_module.emitter_on_off', 'bool', True)
            if ok and restored:
                self._cur_exp, self._cur_gain = float(exp), float(gain)
                self.get_logger().info(
                    f'brightness governor: mean={self._ema:.0f} -> '
                    f'exposure={exp}us gain={gain}')
            else:
                self.get_logger().warn(
                    f'brightness governor: param set failed '
                    f'(ok={ok} alternation_restored={restored})')
        finally:
            self._ema = None  # resample fresh after the change
            self._gov_cooldown_until = time.monotonic() + 15.0
            self._gov_busy = False

    # --- depth path -----------------------------------------------------------

    def _on_depth(self, msg):
        k = self._key(msg)
        with self._lock:
            mode = self._d_modes.pop(k, None)
            if mode is None:
                self._d_imgs[k] = msg
                _trim(self._d_imgs)
                return
        if mode == 1:
            self.pub_d.publish(msg)
            self._depth_count += 1

    def _on_depth_meta(self, msg):
        k = self._key(msg)
        mode = self._mode_of(msg)
        with self._lock:
            img = self._d_imgs.pop(k, None)
            if img is None:
                self._d_modes[k] = mode
                _trim(self._d_modes)
                return
        if mode == 1:
            self.pub_d.publish(img)
            self._depth_count += 1

    def _report(self):
        self.get_logger().info(
            f'split output Hz: stereo pairs {self._pair_count / 10.0:.1f}, '
            f'depth {self._depth_count / 10.0:.1f}')
        self._pair_count = 0
        self._depth_count = 0


def main():
    rclpy.init()
    node = EmitterSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
