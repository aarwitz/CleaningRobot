#!/usr/bin/env python3
"""Honest pi0 evaluator -- the anti-"5/5" harness (HANDOFF 5.5).

Doctrine, enforced structurally:
  * verify ONLY from the standard high pose, via the claw-region pixel-diff
    against an empty-claw reference captured at session start (the same
    detector-free check the teacher uses; detectors lie about the claw).
  * a trial is VOID unless staging is confirmed BEFORE the policy runs:
    head_scout must see a target inside the reach annulus. Voids are
    reported separately and never count as misses or holds.
  * the policy stop is log-verified (a stop that is not confirmed leaves
    execute mode silently armed -- 2026-07-24).

Per trial: scout staging check -> /pi/request execute -> watch for
watch_s -> stop (verified) -> high pose -> claw-diff verify -> if held,
release the object back into the annulus. Results to a jsonl.

Invoke via `robot eval` only.
"""
import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import rclpy
from std_msgs.msg import String

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from pick_pipeline import PickPipeline, FLOOR_REACH_R
from sock_cycle import GRIP_CLOSED

WIDE = 0.35


def stop_verified(c):
    """Send mode:stop and confirm from the bridge's own log."""
    c.pi_pub.publish(String(data=json.dumps({'mode': 'stop'})))
    c.spin(1.5)
    try:
        log = subprocess.run(
            ['tail', '-5', '/tmp/pi_bridge.log'],
            capture_output=True, text=True).stdout
    except Exception:
        log = ''
    return 'request: stop -> mode=idle' in log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prompt', default='pick up the sock')
    ap.add_argument('--trials', type=int, default=3)
    ap.add_argument('--watch-s', type=float, default=40.0)
    ap.add_argument('--out', default='/demos/pi_evals.jsonl')
    a = ap.parse_args()

    rclpy.init()
    c = PickPipeline()
    c.enable_wrist_yolo()           # head_yolo sub for the staging scout
    c.pi_pub = c.create_publisher(String, '/pi/request', 5)
    if not c.wait_ready() or not c.wait_depth():
        print('ABORT: teleop/camera/depth not ready')
        return 1
    # empty-claw reference, high pose (same as the teacher)
    c.move(255.0, 0.0, 60.0, t=GRIP_CLOSED, speed=90.0)
    c.spin(1.2)
    c.claw_ref = c.wrist.copy() if c.wrist is not None else None
    if c.claw_ref is None:
        print('ABORT: no wrist frame for the empty-claw reference')
        return 1

    held_n = miss_n = void_n = 0
    for t in range(a.trials):
        print(f'\n== trial {t + 1}/{a.trials} ==')
        # 1. staging: a target must exist in the annulus BEFORE the policy
        targets = [tg for tg in c.head_scout(a.prompt)
                   if 170.0 <= math.hypot(tg[0], tg[1]) <= FLOOR_REACH_R]
        rec = {'ts': time.time(), 'trial': t, 'prompt': a.prompt}
        if not targets:
            print('  STAGING FAILED -- trial VOID (does not count)')
            void_n += 1
            rec['void'] = True
            with open(a.out, 'a') as f:
                f.write(json.dumps(rec) + '\n')
            continue
        tx, ty = targets[0][0], targets[0][1]
        rec['staged_target'] = [round(tx), round(ty)]
        print(f'  staged: target at ({tx:.0f},{ty:.0f})')
        # 2. run the policy
        c.pi_pub.publish(String(data=json.dumps(
            {'prompt': a.prompt, 'mode': 'execute'})))
        print(f'  policy running ({a.watch_s:.0f}s)...')
        c.spin(a.watch_s)
        # 3. stop, verified
        stopped = stop_verified(c)
        rec['stop_verified'] = stopped
        if not stopped:
            print('  WARNING: stop NOT confirmed in pi_bridge log')
        # 4. verify from the high pose (claw-diff; verify() commands it)
        held = c.verify(a.prompt, pick_xy=(tx, ty))
        rec['held'] = bool(held)
        held_n += held
        miss_n += not held
        with open(a.out, 'a') as f:
            f.write(json.dumps(rec) + '\n')
        # 5. release back into the annulus for the next trial
        if held:
            c.move(280.0, 0.0, -100.0, t=GRIP_CLOSED, speed=70.0)
            c.grip(WIDE, secs=0.8)
            c.move(255.0, 0.0, 60.0, t=WIDE, speed=90.0)

    print(f'\n== pi0 eval: {held_n} held / {miss_n} miss '
          f'/ {void_n} void of {a.trials} trials ==')
    print(f'  (voids excluded from the rate; log: {a.out})')
    c.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
