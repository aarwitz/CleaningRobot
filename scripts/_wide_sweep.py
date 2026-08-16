#!/usr/bin/env python3
"""Wide-open sweep pick with band-aware held detection.

Technique (operator-validated): open the gripper VERY wide (0.35) well before
approaching, come in from the +y (robot-left) side, and sweep in toward the
ball while closing — never descend straight onto it with a narrow/closed claw
or the plate knocks it away.

With rubber bands on the jaws the old torque/gap held() is blind: an EMPTY
close stalls at 2.250 with torH 288 (the claw squeezing its own bands). The
discriminator is now the STALL ANGLE: a 60mm ball between the pads stops the
claw far earlier. held := measured claw angle < 2.10.
"""
import sys
import rclpy
sys.path.insert(0, '/scripts')
import sock_cycle
sock_cycle.GRIP_OPEN = 0.35             # wide envelop, per operator technique
from ball_pick import BallPick, load_cal

WIDE = 0.35
CLOSE = 2.60
HELD_ANGLE_MAX = 2.10                   # empty-with-bands stalls at 2.250


def band_held(c):
    a = (c.state or {}).get('arm') or {}
    ta = a.get('t')
    return (ta is not None and ta < HELD_ANGLE_MAX), ta, abs(a.get('torH') or 0)


rclpy.init()
c = BallPick()
assert c.wait_ready() and c.wait_depth()
tx, ty = load_cal()
est, why = c.observe_ball(tx, ty)
if est is None:
    print(f'no localization: {why}')
    sys.exit(2)
bx, by, _ = est
import math
r = math.hypot(bx, by)
if not (200.0 <= r <= 420.0):
    print(f'estimate ({bx:.0f},{by:.0f}) r={r:.0f} outside sane pick band -- '
          'refusing to flail (and NOT driving the base)')
    sys.exit(3)
gz = -178.0
print(f'sweep pick at ({bx:.0f},{by:.0f}) gz {gz}')
c.move(bx, by + 55.0, gz + 90.0, WIDE, speed=110.0, settle=False)
c.move(bx, by + 55.0, gz + 25.0, speed=60.0, settle=False)
c.sweep_in(bx, by, gz, 55.0, 8.0, CLOSE, close_start=0.12, z_blend=25.0)
ok, ta, th = band_held(c)
print(f'close : held={ok} stall={ta:.3f} torH={th}')
if ok:
    c.move(bx, by - 8.0, gz + 40.0, speed=25.0)
    ok, ta, th = band_held(c)
    print(f'lift40: held={ok} stall={ta:.3f} torH={th}')
if ok:
    c.move(bx, by - 8.0, gz + 120.0, speed=30.0)
    c.spin(0.6)
    ok, ta, th = band_held(c)
    print(f'liftup: held={ok} stall={ta:.3f} torH={th}')
print('WIDE SWEEP PICK ' + ('OK -- HOLDING' if ok else 'FAILED'))
c.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 2)
