#!/usr/bin/env python3
"""Multi-object "put the X in the bin" demo collector.

This is the successor to sock_cycle.py and the data engine for the apartment-
cleanup skill. Differences that matter for training:

1. EPISODES END WITH A RELEASE INTO A CONTAINER. The sock dataset's episodes
   were pick-AND-place cycles all labelled "pick up the sock" — the language
   never matched the behaviour, every place point doubled as the next pick
   point, and the policy had no mode for "let go and withdraw". Here every
   episode is pick -> transit -> drop into the bin -> retreat, and the prompt
   names that whole behaviour.

2. PROMPTS VARY WITH THE OBJECT. The detector (GroundingDINO, open-vocab)
   labels each object the operator feeds in, and the prompt becomes
   "put the {label} in the bin". With a single constant prompt the model can
   ignore language entirely; varying object + prompt jointly is what makes the
   text input load-bearing.

3. GROUND TRUTH VIA STAGING. The operator drops the next object roughly on the
   feed mark; an UNRECORDED staging pick moves it to a random script-chosen
   point (exact coordinates, position variety for free), and only the pick
   from that known point is recorded. Operator placement error never enters
   the dataset; the recorded miss-retry path covers the rest.

Operator workflow (sit next to the bin):
  - object appears on the feed mark -> robot stages it, records the episode,
    drops it in the bin
  - take any object out of the bin, scrunch it, put it on the feed mark
  - repeat; mix the objects freely

Setup, once per session (arm on, teleop up, RSL tunnel up):
  python3 /scripts/bin_cycle.py --calibrate-bin        # jog claw over bin rim
  python3 /scripts/bin_cycle.py --test-drop --grasp-z -121   # one dry cycle
  python3 /scripts/bin_cycle.py --episodes 40 --grasp-z -121 --feed 285,10
"""
import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import cv2
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sock_cycle import (Cycle, Recorder, VisionCal, cycle,          # noqa: E402
                        GRIP_OPEN, GRIP_CLOSED, R_MAX, clamp)
try:
    import dino_client
except Exception:
    dino_client = None

# Open-vocab feed-spot prompt. The area band is wider than the sock one:
# a plush dog toy or scrunched towel runs bigger than a sock wad.
DETECT_PROMPT = ('sock,rolled sock,cloth,folded cloth,towel,shirt,'
                 'plush toy,stuffed toy,dog toy,rope toy')
DETECT_AREA = (2500, 60000)

# Detector label -> the noun used in the training prompt. Anything unmapped
# falls back to its own first word, so new detector phrasings degrade softly.
LABEL_NOUN = {
    'sock': 'sock', 'rolled sock': 'sock',
    'cloth': 'cloth', 'folded cloth': 'cloth', 'towel': 'towel',
    'shirt': 'shirt',
    'plush toy': 'dog toy', 'stuffed toy': 'dog toy',
    'dog toy': 'dog toy', 'rope toy': 'dog toy',
}


def detect_labeled(c, tries=2):
    """(noun, center_px) of the top detection at the current pose, or None."""
    if dino_client is None or c.img is None:
        return None
    for _ in range(tries):
        cv2.imwrite('/tmp/_dino_frame.png', c.img)
        try:
            dets = dino_client.detect('/tmp/_dino_frame.png', DETECT_PROMPT,
                                      roi=dino_client.SOCK_ROI,
                                      area=DETECT_AREA)
        except Exception as e:
            print(f'  [vision] detect failed: {e}')
            dets = []
        if dets:
            raw = (dets[0].get('label') or 'object').strip().lower()
            noun = LABEL_NOUN.get(raw, raw.split(',')[0].split()[-1] or 'object')
            return noun, dino_client.box_center(dets[0]['box'])
        c.spin(0.4)
    return None


def await_object(c, gz_hover, poll=1.5):
    """Park at the observe pose and wait for the operator to feed an object.

    Requires TWO consecutive detections with a stable center (<18 px apart)
    so we never launch a pick at the operator's hand mid-placement."""
    c.move(c.OBSERVE[0], c.OBSERVE[1], gz_hover, GRIP_OPEN, speed=110.0)
    print('  waiting for the next object on the feed mark... (Ctrl-C to stop)')
    prev = None
    while True:
        c.spin(poll)
        d = detect_labeled(c, tries=1)
        if d is None:
            prev = None
            continue
        noun, ctr = d
        if prev and prev[0] == noun and math.dist(prev[1], ctr) < 18.0:
            print(f'  object: {noun} at px ({ctr[0]:.0f},{ctr[1]:.0f})')
            return noun, ctr
        prev = (noun, ctr)


def pick_with_retries(c, px, py, gz, hover, rec, close_to=GRIP_CLOSED,
                      approach_dy=55.0, overshoot=6.0, close_start=0.12,
                      z_blend=25.0, max_retries=2, reacquire=None):
    """The sweep-in pick with RECORDED miss-recovery (same shape as
    sock_cycle.cycle's pick half). Returns (ok, why, tor, px, py)."""
    tor, retries = {}, 0
    while True:
        c.move(px, py + approach_dy, gz + hover, GRIP_OPEN, speed=130.0,
               settle=False, rec=rec, phase='approach')
        c.move(px, py + approach_dy, gz + z_blend, speed=70.0,
               settle=False, rec=rec, phase='descend')
        c.sweep_in(px, py, gz, approach_dy, overshoot, close_to,
                   close_start=close_start, z_blend=z_blend, rec=rec,
                   phase='grasp')
        ok, th, ta, gap = c.held()
        tor['close'] = {'torH': th, 't': round(ta, 3), 'gap': round(gap, 3)}
        if ok:
            c.move(px, py - overshoot, gz + hover, speed=90.0, settle=False,
                   rec=rec, phase='lift')
            ok, th, ta, gap = c.held()
            tor['lift'] = {'torH': th, 't': round(ta, 3), 'gap': round(gap, 3)}
        if ok:
            tor['retries'] = retries
            return True, 'ok', tor, px, py
        if retries >= max_retries:
            return False, (f'miss after {retries} retries (torH {th}, '
                           f'gap {gap:.3f})'), tor, px, py
        retries += 1
        tor[f'miss_{retries}'] = {'torH': th, 't': round(ta, 3),
                                  'gap': round(gap, 3)}
        c.grip(GRIP_OPEN, secs=0.6, rec=rec, phase='recover')
        c.move(px, py + approach_dy * 0.6, gz + hover, speed=100.0,
               settle=False, rec=rec, phase='recover')
        if reacquire is not None:
            pt = reacquire()
            if pt is not None:
                print(f'  [retry {retries}] re-acquired at '
                      f'({pt[0]:.0f},{pt[1]:.0f})')
                px, py = pt


def drop_in_bin(c, gz, hover, bin_pose, rec):
    """Transit from the post-lift hover to the bin and release. The travel
    height is the higher of the pick hover and the bin drop height, so the
    claw always clears the rim."""
    bx, by, bz = bin_pose
    zt = max(gz + hover, bz)
    x0, y0 = c.last_cmd[0], c.last_cmd[1]
    if zt > gz + hover + 1.0:
        c.move(x0, y0, zt, speed=90.0, settle=False, rec=rec, phase='lift')
    c.move(bx, by, zt, speed=130.0, settle=False, rec=rec, phase='transit')
    c.move(bx, by, bz, speed=60.0, settle=False, rec=rec, phase='over_bin')
    c.grip(GRIP_OPEN, secs=0.9, rec=rec, phase='drop')
    dropped_torh = c.torh()
    c.move(bx, by, zt, speed=90.0, settle=False, rec=rec, phase='rise')
    c.move(c.OBSERVE[0], c.OBSERVE[1], gz + hover, speed=120.0,
           settle=False, rec=rec, phase='retreat')
    c.settle(c.OBSERVE[0], c.OBSERVE[1], gz + hover, rec=rec, phase='retreat')
    return dropped_torh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=1)
    ap.add_argument('--grasp-z', type=float, default=-121.0)
    ap.add_argument('--hover', type=float, default=85.0)
    ap.add_argument('--feed', type=str, default=None,
                    help='x,y of the feed mark (default: arm pose at start)')
    ap.add_argument('--out', type=str, default='/demos_bin')
    ap.add_argument('--cal-file', type=str, default='/demos/vision_cal.json',
                    help='px->mm map from sock_cycle --calibrate-vision '
                         '(pose-dependent only; reusable across sessions)')
    ap.add_argument('--bin-file', type=str, default='/demos_bin/bin_cal.json')
    ap.add_argument('--no-stage', action='store_true',
                    help='skip the unrecorded staging pick; record straight '
                         'from the (vision-corrected) feed mark')
    ap.add_argument('--stage-spread', type=float, default=28.0,
                    help='mm half-range of the random staged pick point '
                         'around the feed mark (position variety)')
    ap.add_argument('--close-to', type=float, default=GRIP_CLOSED)
    ap.add_argument('--approach-dy', type=float, default=55.0)
    ap.add_argument('--overshoot', type=float, default=6.0)
    ap.add_argument('--close-start', type=float, default=0.12)
    ap.add_argument('--z-blend', type=float, default=25.0)
    ap.add_argument('--max-retries', type=int, default=2)
    ap.add_argument('--calibrate-bin', action='store_true',
                    help='interactively jog the (closed) claw until it hovers '
                         'centered just above the bin rim, then save')
    ap.add_argument('--test-drop', action='store_true',
                    help='one full unrecorded cycle: feed-mark pick -> bin')
    ap.add_argument('--start', type=int, default=None)
    a = ap.parse_args()

    rclpy.init()
    c = Cycle()
    if not c.wait_ready():
        print('FAIL: no /teleop/state, no camera, or teleop_node not subscribed')
        return 1
    if c.estopped():
        print('FAIL: E-STOP engaged -- clear it in the console first')
        return 1
    p = c.pose()
    print(f'arm at ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) grip={p[3]:.2f}')

    root = Path(a.out)
    root.mkdir(parents=True, exist_ok=True)
    bin_path = Path(a.bin_file)

    # ── bin calibration: the claw is the pointer ────────────────────────────
    if a.calibrate_bin:
        pose = [170.0, 200.0, 0.0]
        if bin_path.exists():
            pose = json.loads(bin_path.read_text())['pose']
        c.move(*pose, GRIP_CLOSED, speed=60.0)
        print('Jog the claw until it is centered over the bin, just above the '
              'rim.\nCommands: "x -10" / "y 15" / "z -20" (mm, relative), '
              '"ok" to save, "q" to abort.')
        while True:
            try:
                line = input('> ').strip().lower()
            except EOFError:
                line = 'q'
            if line == 'ok':
                bin_path.parent.mkdir(parents=True, exist_ok=True)
                bin_path.write_text(json.dumps(
                    {'pose': list(c.last_cmd), 'ts': time.time()}))
                print(f'bin saved: {[round(v,1) for v in c.last_cmd]} '
                      f'-> {bin_path}')
                break
            if line == 'q':
                print('aborted, nothing saved')
                break
            try:
                ax, dv = line.split()
                i = 'xyz'.index(ax)
                tgt = list(c.last_cmd)
                tgt[i] += float(dv)
                if math.hypot(tgt[0], tgt[1]) > R_MAX - 2:
                    print(f'  r would exceed {R_MAX}; move the bin closer in')
                    continue
                c.move(*tgt, speed=45.0)
                print(f'  now {[round(v,1) for v in c.last_cmd]}')
            except (ValueError, IndexError):
                print('  ?  ("x -10", "ok", "q")')
        c.destroy_node(); rclpy.shutdown()
        return 0

    if not bin_path.exists():
        print(f'no {bin_path} -- run --calibrate-bin first')
        return 2
    bin_pose = json.loads(bin_path.read_text())['pose']
    print(f'bin at ({bin_pose[0]:.0f}, {bin_pose[1]:.0f}, drop z '
          f'{bin_pose[2]:.0f})')

    feed = tuple(float(v) for v in a.feed.split(',')) if a.feed else (p[0], p[1])
    print(f'feed mark ({feed[0]:.0f}, {feed[1]:.0f}), grasp z {a.grasp_z}')

    cal = VisionCal(a.cal_file)
    if cal.A is None:
        print('WARNING: no vision cal -- feed-mark corrections disabled; '
              'operator placement must be accurate')

    gz, hover = a.grasp_z, a.hover

    def reacquire_around(anchor):
        def f():
            c.move(c.OBSERVE[0], c.OBSERVE[1], gz + hover, speed=110.0)
            c.spin(0.5)
            d = detect_labeled(c)
            if d is None or cal.A is None or cal.ref_px is None:
                return None
            cal.set_reference(anchor, cal.ref_px)
            return cal.correct(d[1])
        return f

    if a.test_drop:
        noun, ctr = await_object(c, gz + hover)
        ok, why, tor, px, py = pick_with_retries(
            c, feed[0], feed[1], gz, hover, None, close_to=a.close_to,
            approach_dy=a.approach_dy, overshoot=a.overshoot,
            close_start=a.close_start, z_blend=a.z_blend,
            max_retries=a.max_retries)
        if not ok:
            print(f'TEST FAILED at pick: {why}')
            c.destroy_node(); rclpy.shutdown(); return 2
        th = drop_in_bin(c, gz, hover, bin_pose, None)
        print(f'TEST DROP done (post-drop torH {th}); check the {noun} '
              'is in the bin')
        c.destroy_node(); rclpy.shutdown()
        return 0

    # ── collection ───────────────────────────────────────────────────────────
    idx = a.start if a.start is not None else (
        1 + max([int(d.name[3:]) for d in root.glob('ep_*')] or [-1]))
    ok_n = fail_n = retry_ok_n = 0
    counts = {}

    for k in range(a.episodes):
        noun, ctr = await_object(c, gz + hover)
        px, py = feed
        # y-only correction of the operator's placement vs the feed mark:
        # ref anchors at the mark, using the FIRST session detection there.
        if cal.A is not None:
            if cal.ref_px is None:
                cal.set_reference(feed, ctr)    # first object defines ref px
            else:
                cal.set_reference(feed, cal.ref_px)
                corr = cal.correct(ctr)
                if corr is not None and abs(corr[1] - py) > 4.0:
                    print(f'  [vision] feed y {py:.0f} -> {corr[1]:.0f}')
                    px, py = corr

        if not a.no_stage:
            # UNRECORDED staging pick: move the object to an exactly-known
            # random point. Operator placement error dies here, not in data.
            s = a.stage_spread
            tgt = (clamp(feed[0] + random.uniform(-s, s), 180.0, R_MAX - 15),
                   feed[1] + random.uniform(-s, s))
            if math.hypot(*tgt) > R_MAX - 10:
                tgt = feed
            ok, why, tor, _ = cycle(c, px, py, gz, hover, None, place=tgt,
                                    close_to=a.close_to,
                                    approach_dy=a.approach_dy,
                                    overshoot=a.overshoot,
                                    close_start=a.close_start,
                                    z_blend=a.z_blend,
                                    max_retries=a.max_retries,
                                    reacquire=reacquire_around(feed))
            if not ok:
                print(f'  staging pick failed ({why}) -- re-place the object')
                continue
            px, py = tgt

        prompt = f'put the {noun} in the bin'
        rec = Recorder(root, idx, prompt)
        try:
            ok, why, tor, px, py = pick_with_retries(
                c, px, py, gz, hover, rec, close_to=a.close_to,
                approach_dy=a.approach_dy, overshoot=a.overshoot,
                close_start=a.close_start, z_blend=a.z_blend,
                max_retries=a.max_retries,
                reacquire=reacquire_around((px, py)))
            if ok:
                drop_torh = drop_in_bin(c, gz, hover, bin_pose, rec)
                tor['drop'] = drop_torh
        except RuntimeError as e:
            rec.discard()
            print(f'ep {idx}: ABORT {e}')
            break
        extra = {'object': noun, 'pick': [px, py, gz], 'bin': bin_pose,
                 'torque': tor}
        if ok:
            nret = tor.get('retries', 0)
            rec.finish(True, extra)
            ok_n += 1; retry_ok_n += (1 if nret else 0)
            counts[noun] = counts.get(noun, 0) + 1
            print(f'ep {idx:4d}: OK{"+"+str(nret)+"r" if nret else "   "} '
                  f'"{prompt}"  {rec.n:3d} frames')
        else:
            extra['failure'] = why
            rec.finish(False, extra)
            fail_n += 1
            print(f'ep {idx:4d}: FAIL {why}  [kept, success=false]')
            c.grip(GRIP_OPEN, secs=0.6)
        idx += 1

    print(f'\n{ok_n} ok ({retry_ok_n} with recorded recoveries), '
          f'{fail_n} failed-kept -> {root}')
    print('per object: ' + ', '.join(f'{k}: {v}' for k, v in
                                     sorted(counts.items())))
    c.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
