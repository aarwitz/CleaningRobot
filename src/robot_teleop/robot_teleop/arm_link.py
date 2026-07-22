"""Serial link to the Waveshare RoArm M2-S, built for CONTINUOUS teleop.

Why this exists instead of reusing arm_bridge's `_send`
-------------------------------------------------------
arm_bridge drives the arm with `T:104` (CMD_XYZT_GOAL_CTRL). The vendor docs
say of that command: "This command may cause the movement to be blocked."
Measured 2026-07-22: streaming T:104 at 10-20 Hz WEDGES the ESP32 firmware
outright — it stops answering T:105 entirely and only a DTR/RTS reset pulse
brings it back. Interleaving a T:105 feedback poll with every T:104 wedges it
even faster (replies degraded to 9/30 before dying).

So this link never streams T:104. It uses the two commands the vendor
actually documents for continuous control:

  T:123  CMD_CONSTANT_CTRL   velocity jog: {"m":1,"axis":1..4,"cmd":0|1|2,"spd":0..20}
                             m=1 cartesian (axis 1=X 2=Y 3=Z 4=gripper),
                             cmd 0=STOP 1=INCREASE 2=DECREASE.
                             ONE message per velocity change — the firmware
                             does the continuous motion itself.
  T:1041 CMD_XYZT_DIRECT_CTRL non-blocking absolute setpoint. Survives 20 Hz
                             streaming (verified) — used for homing/goto.

Measured speed calibration (2026-07-22, cartesian mode, 1.5 s jogs):
    spd  1 ->  7.8 mm/s      spd  5 -> 42.3 mm/s
    spd  3 -> 25.2 mm/s      spd 10 -> 86.6 mm/s
i.e. ~8.7 mm/s per spd unit, near-perfectly linear. MM_S_PER_SPD encodes it.

SAFETY: a T:123 jog runs FOREVER until an explicit STOP. Every path that can
lose the operator (watchdog timeout, disconnect, shutdown, exception) MUST
call stop_all(). That is the whole reason the node has a deadman.
"""

import json
import threading
import time

try:
    import serial
except ImportError:  # pragma: no cover - hardware dep
    serial = None


# Cartesian-mode axis ids for T:123 (m=1).
AXIS_X, AXIS_Y, AXIS_Z, AXIS_GRIP = 1, 2, 3, 4
AXES = (AXIS_X, AXIS_Y, AXIS_Z, AXIS_GRIP)

CMD_STOP, CMD_INC, CMD_DEC = 0, 1, 2

# Calibration: cartesian mm/s delivered per unit of T:123 `spd`.
MM_S_PER_SPD = 8.7
SPD_MAX = 20            # vendor-suggested ceiling

# Gripper joint limits in radians (measured; T:104 `t` clamps to this).
GRIP_OPEN, GRIP_CLOSED = 1.08, 3.14


class ArmLink:
    """Thread-safe serial link. All public methods may be called from any thread."""

    def __init__(self, port='/dev/ttyUSB0', baud=115200, logger=None):
        self.port_name = port
        self.baud = baud
        self.log = logger
        self.ser = None
        self._lock = threading.RLock()
        # Last jog state actually ON THE WIRE per axis, so we only transmit on
        # change. Steady jogging then costs zero serial traffic.
        self._sent = {a: (CMD_STOP, 0) for a in AXES}
        self.last_fb = None
        self.last_fb_time = 0.0
        self.link_state = 'down'      # down | ok | stale | recovering

    # ── wire ────────────────────────────────────────────────────────────
    def open(self):
        if serial is None:
            self._err('pyserial missing — arm link disabled')
            return False
        with self._lock:
            try:
                # dsrdtr=None + RTS/DTR low: vendor convention that stops the
                # ESP32 auto-resetting the moment we open the port.
                self.ser = serial.Serial(self.port_name, self.baud,
                                         timeout=0.15, dsrdtr=None)
                self.ser.setRTS(False)
                self.ser.setDTR(False)
                time.sleep(0.6)
                self.ser.reset_input_buffer()
                self.link_state = 'ok'
                self._info(f'arm link open on {self.port_name}')
                return True
            except Exception as e:
                self._err(f'arm serial open failed: {e}')
                self.ser = None
                return False

    def close(self):
        with self._lock:
            if self.ser:
                try:
                    self.stop_all()
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.link_state = 'down'

    def _write(self, obj):
        with self._lock:
            if not self.ser:
                return False
            try:
                self.ser.write(json.dumps(obj).encode() + b'\n')
                self.ser.flush()
                return True
            except Exception as e:
                self._err(f'arm write failed: {e}')
                self.link_state = 'down'
                return False

    # ── feedback ────────────────────────────────────────────────────────
    def poll_feedback(self, timeout=0.25):
        """Request T:105 and parse the T:1051 reply, skipping command echoes.

        Keep the CALL RATE modest (<=10 Hz). The arm echoes everything it
        receives, so an over-eager poll loop both floods the link and starves
        the firmware's own servo loop.
        """
        with self._lock:
            if not self.ser:
                return None
            try:
                self.ser.reset_input_buffer()
                self.ser.write(b'{"T":105}\n')
                self.ser.flush()
                deadline = time.time() + timeout
                while time.time() < deadline:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    try:
                        d = json.loads(line)
                    except Exception:
                        continue          # boot banner / echo / partial line
                    if d.get('T') == 1051:
                        self.last_fb = d
                        self.last_fb_time = time.time()
                        self.link_state = 'ok'
                        return d
            except Exception as e:
                self._err(f'arm read failed: {e}')
                self.link_state = 'down'
        return None

    def fb_age(self):
        return time.time() - self.last_fb_time if self.last_fb_time else 1e9

    # ── motion ──────────────────────────────────────────────────────────
    def jog(self, axis, mm_per_s, force=False):
        """Set a continuous cartesian velocity on one axis.

        mm_per_s is signed; 0 stops the axis. Only transmits when the
        (direction, spd) pair actually changes, so holding a key steady sends
        nothing after the initial ramp.
        """
        spd = int(round(abs(mm_per_s) / MM_S_PER_SPD))
        spd = max(0, min(SPD_MAX, spd))
        if spd == 0:
            cmd = CMD_STOP
        else:
            cmd = CMD_INC if mm_per_s > 0 else CMD_DEC
        if not force and self._sent.get(axis) == (cmd, spd):
            return
        self._sent[axis] = (cmd, spd)
        self._write({'T': 123, 'm': 1, 'axis': axis, 'cmd': cmd, 'spd': spd})

    def jog_grip(self, rate, force=False):
        """Gripper jog. rate>0 closes, rate<0 opens (matches `t` increasing)."""
        spd = max(0, min(SPD_MAX, int(round(abs(rate)))))
        cmd = CMD_STOP if spd == 0 else (CMD_INC if rate > 0 else CMD_DEC)
        if not force and self._sent.get(AXIS_GRIP) == (cmd, spd):
            return
        self._sent[AXIS_GRIP] = (cmd, spd)
        self._write({'T': 123, 'm': 1, 'axis': AXIS_GRIP, 'cmd': cmd, 'spd': spd})

    def stop_axis(self, axis):
        self._sent[axis] = (CMD_STOP, 0)
        self._write({'T': 123, 'm': 1, 'axis': axis, 'cmd': CMD_STOP, 'spd': 0})

    def stop_all(self):
        """Unconditional STOP on every axis. Never suppressed by change-detection —
        this is the deadman path and must always reach the wire."""
        for a in AXES:
            self._sent[a] = (CMD_STOP, 0)
            self._write({'T': 123, 'm': 1, 'axis': a, 'cmd': CMD_STOP, 'spd': 0})

    def goto(self, x, y, z, t):
        """Absolute move via the NON-BLOCKING T:1041. Never use T:104 here."""
        self._write({'T': 1041, 'x': float(x), 'y': float(y),
                     'z': float(z), 't': float(t)})

    # ── auxiliary ───────────────────────────────────────────────────────
    def led(self, value):
        self._write({'T': 114, 'led': int(max(0, min(255, value)))})

    def torque(self, on):
        """T:210. Torque OFF lets the arm be moved by hand (kinesthetic guiding)."""
        self._write({'T': 210, 'cmd': 1 if on else 0})

    def reset_firmware(self):
        """DTR/RTS pulse to reboot the ESP32 — the ONLY recovery from a wedge.

        The arm drops to its middle pose on reboot, so callers must re-sync any
        cached target afterwards. Takes ~3.5 s to come back.
        """
        with self._lock:
            if not self.ser:
                return False
            self.link_state = 'recovering'
            self._info('resetting arm firmware (DTR/RTS pulse)')
            try:
                self.ser.dtr = False
                self.ser.rts = True
                time.sleep(0.15)
                self.ser.rts = False
                time.sleep(0.05)
                self.ser.dtr = False
                time.sleep(3.5)
                self.ser.reset_input_buffer()
                self._sent = {a: (CMD_STOP, 0) for a in AXES}
                return True
            except Exception as e:
                self._err(f'arm reset failed: {e}')
                return False

    # ── logging shims ───────────────────────────────────────────────────
    def _info(self, m):
        if self.log:
            self.log.info(m)

    def _err(self, m):
        if self.log:
            self.log.error(m)
