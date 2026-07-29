#!/usr/bin/env python3
"""Two-stage, two-camera pick pipeline — the generalizable core.

Sensors are used for what they are actually good at:
  COARSE  head RealSense + GroundingDINO(RSL): open-vocab detect + aligned
          depth near-cluster -> 3D target in arm frame. Reliable 0.3-1.0m,
          blind closer. Drives approach/staging.
  REFINE  wrist global-shutter RGB + DINO: rigid to the EE, so from the
          canonical pre-grasp hover the object's pixel offset from the GRASP
          ANCHOR pixel converts to an arm-frame correction with a constant
          mm/px scale. No depth needed; works exactly where the head cam is
          blind. The anchor SELF-CALIBRATES: each successful pick logs the
          object's pre-grasp pixel; the running mean becomes the anchor
          (wrist_anchor.json).
  VERIFY  both cameras, vision only: head cam floor-clear + wrist cam
          object-at-claw. No grip-signal heuristics (rubber bands made stall
          angle and torque posture-dependent liars).

Grasp strategies are a TABLE, not scripts:
  sweep   soft/compressible (sock, cloth, plush rabbit): wide open, descend
          fully on the +y side, flat lateral sweep while closing.
  cage    rigid rollers (rubber ball): straddle + roll into the fixed jaw.

Episodes record BOTH cameras (frames/ + frames_wrist/) — every episode is a
two-view VLA training sample.

  python3 /scripts/pick_pipeline.py --object rabbit \
      --prompt 'white fluffy rabbit toy. white fur plush toy' \
      --strategy sweep [--record] [--episodes N] [--allow-drive]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sock_cycle
sock_cycle.GRIP_OPEN = 0.35
import dino_client
from ball_pick import BallPick, OBSERVE, load_cal
from behavior_manager_interfaces.srv import DriveRelative
from sensor_msgs.msg import CompressedImage
from sock_cycle import GRIP_CLOSED, Recorder

ROI = (16, 200, 624, 480)
WIDE = 0.35
PICK_R = 300.0                  # proven grasp radius (vertical-claw posture)
ANCHOR_FILE = '/demos/wrist_anchor.json'

STRATEGIES = {
    # gz: wrist z at close; soft piles sit higher than their contact point
    'sweep': dict(gz=-172.0, approach_dy=60.0, overshoot=12.0,
                  close_to=2.90, close_start=0.25, secure=3.05),
    'cage':  dict(gz=-176.0, approach_dy=60.0, overshoot=14.0,
                  close_to=2.60, close_start=0.30, secure=2.95),
}


class PickPipeline(BallPick):
    def __init__(self):
        super().__init__()
        self.wrist = None
        self.create_subscription(CompressedImage,
                                 '/wrist_cam/image_raw/compressed',
                                 self._w, 2)
        self.drv = self.create_client(DriveRelative,
                                      '/motor_controller/drive_relative')

    def _w(self, m):
        self.wrist = cv2.imdecode(np.frombuffer(m.data, np.uint8),
                                  cv2.IMREAD_COLOR)

    # ── LED: off by default; the full-brightness LED at close range blows
    # out the wrist image (operator-observed). Auto-on DIM only when the
    # wrist frame is actually dark, stepped, and off again when bright. ──
    def set_led(self, val):
        if getattr(self, '_led', None) == val:
            return
        from std_msgs.msg import String as _S
        m = _S()
        m.data = f'led:{val}'
        self.act.publish(m)
        self._led = val
        print(f'  [light] led -> {val}')

    def ensure_light(self):
        if self.wrist is None:
            return
        mean = float(np.mean(cv2.cvtColor(self.wrist, cv2.COLOR_BGR2GRAY)))
        led = getattr(self, '_led', 0) or 0
        if mean < 45.0 and led == 0:
            self.set_led(70)
            self.spin(0.6)
        elif mean < 40.0 and led == 70:
            self.set_led(140)
            self.spin(0.6)
        elif mean > 110.0 and led > 0:
            self.set_led(0)
            self.spin(0.3)

    # ── coarse: head cam + depth ────────────────────────────────────────────
    def coarse(self, prompt):
        """(x, y, status, u): 3D target in arm mm from head cam, with the
        localization guards learned the hard way."""
        self.move(*OBSERVE, t=GRIP_CLOSED, speed=100.0)
        self.spin(0.7)
        cv2.imwrite('/tmp/_head.png', self.img)
        try:
            dets = dino_client.detect('/tmp/_head.png', prompt,
                                      confidence=0.25, roi=ROI)
        except Exception as e:
            return None, None, f'DINO unreachable: {e}', None
        if not dets:
            return None, None, 'no detection', None
        box, score = dets[0]['box'], dets[0]['score']
        u = int((box[0] + box[2]) / 2)
        x0, y0, x1, y1 = (int(t) for t in box)
        d = self.depth[max(0, y0):y1, max(0, x0):x1].astype(float)
        valid = d[(d > 100) & (d < 4000)]
        frac = valid.size / max(d.size, 1)
        if valid.size < 20 or frac < 0.05 or y1 >= 472:
            print(f'  [coarse] box {[round(b) for b in box]} score '
                  f'{score:.2f} -- blind ({100*frac:.1f}%'
                  + (', frame-bottom' if y1 >= 472 else '') + ')')
            return None, None, 'blind', u
        near = valid[valid <= np.percentile(valid, 50)]
        surf = float(np.median(near))
        tx, ty = load_cal()
        fx, _, cx, _ = self.K
        bx = surf + tx
        by = -((u - cx) * surf / fx) + ty
        print(f'  [coarse] {[round(b) for b in box]} score {score:.2f} '
              f'depth {surf:.0f}mm -> arm ({bx:.0f},{by:.0f})')
        self.publish_flywheel(
            self.img, {'stage': 'coarse (head cam)', 'label': prompt.split('.')[0],
                       'score': score, 'box': [round(t) for t in box],
                       'depth_mm': surf, 'valid_px': int(valid.size),
                       'pick_arm': [round(bx), round(by)], 'prompt': prompt},
            box=box, pick_px=(u, int((y0 + y1) / 2)))
        return bx, by, 'ok', u

    def is_claw_det(self, box):
        """The claw's own fingers get detected as objects (seen: 'white
        fluffy rabbit toy' 0.39 on the black finger). The fingers live in a
        fixed region of the wrist image AND are near-black; both together =
        reject."""
        if self.wrist is None:
            return False
        u = (box[0] + box[2]) / 2
        v = (box[1] + box[3]) / 2
        in_claw_zone = 40 < u < 470 and v > 300
        if not in_claw_zone:
            return False
        x0, y0, x1, y1 = (max(0, int(t)) for t in box)
        patch = self.wrist[y0:y1, x0:x1]
        if patch.size == 0:
            return False
        dark = float(np.mean(cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY))) < 70.0
        if dark:
            print(f'  [wrist] rejecting claw self-detection at '
                  f'({u:.0f},{v:.0f})')
        return dark

    # ── refine: wrist cam from the pre-grasp hover ──────────────────────────
    def refine(self, bx, by, prompt, gz):
        """Hover above the coarse target, detect in the WRIST frame, correct
        (bx, by) by the offset from the grasp anchor. Returns possibly
        corrected (bx, by); no-ops gracefully without wrist frames/anchor."""
        self.move(bx, by, gz + 130.0, t=WIDE, speed=90.0, grip_ramp=True)
        self.spin(0.9)
        if self.wrist is None:
            print('  [refine] no wrist frames -- skipping')
            return bx, by, None
        self.ensure_light()
        cv2.imwrite('/tmp/_wrist.png', self.wrist)
        try:
            dets = dino_client.detect('/tmp/_wrist.png', prompt,
                                      confidence=0.35)
            dets = [d_ for d_ in dets if not self.is_claw_det(d_['box'])]
        except Exception:
            dets = []
        if not dets:
            print('  [refine] no wrist detection -- keeping coarse target')
            self.publish_flywheel(self.wrist,
                                  {'stage': 'refine (wrist cam): no det',
                                   'label': prompt.split('.')[0]},
                                  cam='wrist')
            return bx, by, None
        wbox = dets[0]['box']
        wu, wv = (wbox[0] + wbox[2]) / 2, (wbox[1] + wbox[3]) / 2
        anc = load_anchor()
        if anc is None:
            print(f'  [refine] wrist sees it at px ({wu:.0f},{wv:.0f}); no '
                  'anchor yet (self-calibrates from successful picks)')
            self.publish_flywheel(self.wrist,
                                  {'stage': 'refine (wrist cam): no anchor',
                                   'label': prompt.split('.')[0],
                                   'score': dets[0]['score'],
                                   'box': [round(t) for t in wbox]},
                                  box=wbox, pick_px=(wu, wv), cam='wrist')
            return bx, by, (wu, wv)
        # wrist cam rigid to EE: px offset -> arm offset, constant scale.
        # Axes: wrist image x ~ arm -y, image y ~ arm +x (mount-verified).
        dx_mm = (wv - anc['v']) * anc['mm_per_px']
        dy_mm = -(wu - anc['u']) * anc['mm_per_px']
        n = math.hypot(dx_mm, dy_mm)
        if n > 45.0:
            dx_mm, dy_mm = dx_mm * 45.0 / n, dy_mm * 45.0 / n
        print(f'  [refine] wrist px ({wu:.0f},{wv:.0f}) vs anchor '
              f'({anc["u"]:.0f},{anc["v"]:.0f}) -> correct '
              f'({dx_mm:+.0f},{dy_mm:+.0f})mm')
        self.publish_flywheel(self.wrist,
                              {'stage': 'refine (wrist cam)',
                               'label': prompt.split('.')[0],
                               'score': dets[0]['score'],
                               'box': [round(t) for t in wbox],
                               'pick_arm': [round(bx + dx_mm),
                                            round(by + dy_mm)]},
                              box=wbox, pick_px=(wu, wv), cam='wrist')
        return bx + dx_mm, by + dy_mm, (wu, wv)

    # ── grasp strategies ────────────────────────────────────────────────────
    def grasp(self, bx, by, strategy, rec=None):
        p = STRATEGIES[strategy]
        gz = p['gz']
        self.move(bx, by + p['approach_dy'], gz + 110.0, t=WIDE, speed=90.0,
                  grip_ramp=True, rec=rec, phase='approach')
        self.spin(0.8)
        if self.pose()[3] > 1.25:
            print('  gripper failed to open')
            return False
        self.move(bx, by + p['approach_dy'], gz, speed=45.0, settle=False,
                  rec=rec, phase='descend')
        self.sweep_in(bx, by, gz, p['approach_dy'], p['overshoot'],
                      p['close_to'], close_start=p['close_start'],
                      z_blend=0.0, rec=rec, phase='grasp')
        self.grip(p['secure'], secs=0.8, rec=rec, phase='secure')
        self.move(bx, by - p['overshoot'], gz + 40.0, speed=25.0,
                  rec=rec, phase='lift')
        self.move(bx, by - p['overshoot'], gz + 130.0, speed=30.0,
                  rec=rec, phase='lift')
        # verify pose: HIGH and pulled in. At gz+130 a held object hangs at
        # the head camera's own height and appears mid-frame -- right in the
        # floor band -- flagging every good pick as a miss (operator-caught
        # in the first teach session). Up high, held object and floor cannot
        # overlap in the image.
        self.move(270.0, 0.0, 90.0, speed=45.0, rec=rec, phase='verify_pose')
        self.spin(0.8)
        return True

    # ── verify: both cameras, vision only ───────────────────────────────────
    def verify(self, prompt, pick_xy=None):
        """pick_xy restricts the head floor-check to the pick's own bearing:
        other same-class objects in the scene (a second plush) must not
        false-flag 'still on the floor'."""
        held_head = held_wrist = None
        cv2.imwrite('/tmp/_vhead.png', self.img)
        try:
            v = dino_client.detect('/tmp/_vhead.png', prompt,
                                   confidence=0.25, roi=ROI)
            floor = [dv for dv in v
                     if (dv['box'][1] + dv['box'][3]) / 2 > 330]
            if pick_xy is not None and floor and self.K:
                fx, _, cx, _ = self.K
                bx_, by_ = pick_xy
                u_pred = cx - fx * (by_ / max(bx_, 1.0))
                floor = [dv for dv in floor
                         if abs((dv['box'][0] + dv['box'][2]) / 2 - u_pred)
                         < 100]
            held_head = not floor
        except Exception:
            pass
        if self.wrist is not None:
            cv2.imwrite('/tmp/_vwrist.png', self.wrist)
            try:
                vw = dino_client.detect('/tmp/_vwrist.png', prompt,
                                        confidence=0.30)
                vw = [d_ for d_ in vw if not self.is_claw_det(d_['box'])]
                # From the HIGH verify pose the discrimination is easy: a
                # held object is centimeters from the wrist lens (huge bbox);
                # a miss shows distant floor (small/no detection). Area is
                # the signal.
                big = [d_ for d_ in vw
                       if (d_['box'][2] - d_['box'][0]) *
                          (d_['box'][3] - d_['box'][1]) > 18000]
                held_wrist = bool(big)
            except Exception:
                pass
        # both cameras must agree when both have an opinion
        votes = [x for x in (held_head, held_wrist) if x is not None]
        held = bool(votes) and all(votes)
        print(f'  [verify] head floor-clear={held_head} '
              f'wrist holds-object={held_wrist} -> held={held}')
        self.publish_flywheel(
            self.img, {'stage': 'result', 'label': prompt.split('.')[0],
                       'held': held, 'verify_head': held_head,
                       'verify_wrist': held_wrist})
        return held


class DualRecorder(Recorder):
    """Recorder + wrist frames (frames_wrist/) per row."""

    def __init__(self, root, idx, prompt):
        super().__init__(root, idx, prompt)
        (self.dir / 'frames_wrist').mkdir(exist_ok=True)

    def row(self, c, phase, target):
        if getattr(c, 'wrist', None) is not None:
            cv2.imwrite(str(self.dir / 'frames_wrist' / f'{self.n:04d}.jpg'),
                        c.wrist, [cv2.IMWRITE_JPEG_QUALITY, 85])
        super().row(c, phase, target)


BANK_FILE = '/demos/pick_pose_bank.json'


def load_bank():
    p = Path(BANK_FILE)
    return json.loads(p.read_text()) if p.exists() else []


def taught_gz(obj, default):
    """Median grasp z the HUMAN chose for this object, if taught."""
    zs = [e['pose'][2] for e in load_bank()
          if e['object'] == obj and e.get('success')]
    if zs:
        gz = float(np.median(zs))
        print(f'  [bank] using taught gz {gz:.0f} '
              f'({len(zs)} human demos) instead of default {default:.0f}')
        return gz
    return default


def teach(c, a, root):
    """Human-in-the-loop pick-pose demos: the operator teleops the OPEN claw
    into a valid pick pose (web console; e-stop live). On Enter the system
    captures the true pose + what both cameras see, CLOSES, lifts, verifies,
    and learns from it:
      - wrist anchor <- the object's wrist pixel at a HUMAN-vouched grasp
        pose (gold sample; no correction ambiguity)
      - grasp z for this object <- the z the human chose
      - a recorded two-view episode (if --record), success-flagged
    A few of these across object poses bootstrap the autonomous flywheel."""
    import threading
    p_strategy = STRATEGIES[a.strategy]
    n_ok = 0
    while True:
        # record the HUMAN'S approach teleop as part of the demonstration:
        # an input-listener thread lets the main thread keep spinning and
        # capturing frames at ~5Hz while the operator positions the claw
        rec = DualRecorder(root, 1 + max(
            [int(q.name[3:]) for q in root.glob('ep_*')] or [-1]),
            f'pick up the {a.object}') if a.record else None
        done = {'enter': False, 'eof': False}

        def _wait():
            try:
                input(f'\n[teach {a.object}] position the OPEN claw at a '
                      'pick pose, press Enter to capture (Ctrl-C to finish): ')
            except (KeyboardInterrupt, EOFError):
                done['eof'] = True
            done['enter'] = True

        th = threading.Thread(target=_wait, daemon=True)
        th.start()
        while not done['enter']:
            c.spin(0.2)
            if rec:
                p_ = c.pose()
                if p_:
                    rec.row(c, 'human_approach', tuple(p_[:3]) + (p_[3],))
        if done['eof']:
            if rec:
                rec.discard()
            break
        c.spin(0.3)
        pose = c.pose()
        print(f'  pose ({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f}) '
              f'grip {pose[3]:.2f}')
        c.last_cmd = tuple(pose[:3])
        c.grip_cmd = pose[3]
        wrist_px = None
        if c.wrist is not None:
            c.ensure_light()
            cv2.imwrite('/tmp/_teach_wrist.png', c.wrist)
            try:
                dets = dino_client.detect('/tmp/_teach_wrist.png', a.prompt,
                                          confidence=0.30)
                dets = [d_ for d_ in dets if not c.is_claw_det(d_['box'])]
                if dets:
                    b = dets[0]['box']
                    wrist_px = ((b[0]+b[2])/2, (b[1]+b[3])/2)
                    print(f'  wrist sees object at px '
                          f'({wrist_px[0]:.0f},{wrist_px[1]:.0f})')
                    c.publish_flywheel(c.wrist,
                                       {'stage': 'teach: pre-close',
                                        'label': a.object,
                                        'score': dets[0]['score'],
                                        'box': [round(t) for t in b]},
                                       box=b, pick_px=wrist_px, cam='wrist')
            except Exception as e:
                print(f'  wrist detect failed: {e}')
        c.grip(p_strategy['secure'], secs=1.4, rec=rec, phase='grasp')
        c.move(pose[0], pose[1], pose[2] + 130.0, speed=30.0,
               rec=rec, phase='lift')
        # high verify pose: held object at gz+130 sits at head-camera height
        # and lands in the floor band (false MISS on real picks)
        c.move(270.0, 0.0, 90.0, speed=45.0, rec=rec, phase='verify_pose')
        c.spin(0.8)
        held = c.verify(a.prompt, pick_xy=(pose[0], pose[1]))
        if held:
            # return to the taught spot to set it down where it was picked
            c.move(pose[0], pose[1], pose[2] + 120.0, speed=60.0)
        if rec:
            rec.finish(held, {'object': a.object, 'taught': True,
                              'pose': list(pose), 'held': held})
        bank = load_bank()
        bank.append({'object': a.object, 'pose': list(pose),
                     'wrist_px': list(wrist_px) if wrist_px else None,
                     'success': bool(held), 'ts': time.time()})
        Path(BANK_FILE).write_text(json.dumps(bank, indent=1))
        if held:
            n_ok += 1
            if wrist_px is not None:
                update_anchor(*wrist_px)
            print(f'  HELD ✓  (bank now {len(bank)} demos, {n_ok} this '
                  'session). Setting back down.')
            c.move(pose[0], pose[1], pose[2] + 4.0, speed=30.0)
            c.grip(WIDE, secs=1.0)
            c.move(pose[0], pose[1], pose[2] + 120.0, speed=70.0)
        else:
            print('  MISS — claw opened for repositioning')
            c.grip(WIDE, secs=0.8)
    print(f'\nteach session done: {n_ok} successful demos -> {BANK_FILE}')


def load_anchor():
    p = Path(ANCHOR_FILE)
    if p.exists():
        return json.loads(p.read_text())
    return None


def update_anchor(u, v):
    """Running mean over successful picks (the self-calibration)."""
    a = load_anchor() or {'u': u, 'v': v, 'n': 0, 'mm_per_px': 0.55}
    n = a.get('n', 0)
    a['u'] = (a['u'] * n + u) / (n + 1)
    a['v'] = (a['v'] * n + v) / (n + 1)
    a['n'] = n + 1
    Path(ANCHOR_FILE).write_text(json.dumps(a))
    print(f'  [anchor] updated -> ({a["u"]:.0f},{a["v"]:.0f}) n={a["n"]}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--object', required=True)
    ap.add_argument('--prompt', required=True)
    ap.add_argument('--strategy', choices=list(STRATEGIES), default='sweep')
    ap.add_argument('--episodes', type=int, default=1)
    ap.add_argument('--record', action='store_true')
    ap.add_argument('--keep', action='store_true')
    ap.add_argument('--allow-drive', action='store_true',
                    help='permit tucked-arm encoder staging moves (operator '
                         'gate; default OFF)')
    ap.add_argument('--teach', action='store_true',
                    help='human-in-the-loop pick-pose demos: operator '
                         'positions the open claw, system closes/lifts/'
                         'verifies and learns anchor + grasp z')
    ap.add_argument('--out', type=str, default='/demos/picks')
    a = ap.parse_args()

    rclpy.init()
    c = PickPipeline()
    if not c.wait_ready() or not c.wait_depth():
        print('ABORT: teleop/camera/depth not ready')
        return 1
    root = Path(a.out)
    if a.record:
        root.mkdir(parents=True, exist_ok=True)
    if a.teach:
        teach(c, a, root)
        c.destroy_node()
        rclpy.shutdown()
        return 0
    idx = 1 + max([int(p.name[3:]) for p in root.glob('ep_*')] or [-1]) \
        if a.record else 0
    STRATEGIES[a.strategy]['gz'] = taught_gz(a.object,
                                             STRATEGIES[a.strategy]['gz'])

    ok_n = miss_n = 0
    for ep in range(a.episodes):
        print(f'\n── {a.object} attempt {ep+1}/{a.episodes} ──')
        bx, by, why, u0 = c.coarse(a.prompt)
        if why == 'blind' and a.allow_drive:
            # object inside the depth blind zone: tucked-arm backup, coarse
            # again from depth-valid range (the drive-back happens in the
            # normal staging block below)
            print('  [coarse] blind -> tucked backup 0.18m')
            c.move(255, 0, 60, GRIP_CLOSED, speed=90.0)
            fut = c.drv.call_async(DriveRelative.Request(
                dx=-0.18, dy=0.0, dyaw=0.0))
            while not fut.done():
                c.spin(0.1)
            bx, by, why, _ = c.coarse(a.prompt)
        if why != 'ok':
            print(f'  no coarse target ({why})'
                  + ('' if a.allow_drive else ' -- and base moves are gated '
                     'off'))
            miss_n += 1
            continue
        rng, bearing = math.hypot(bx, by), math.atan2(by, bx)
        if rng > 345.0 or abs(by) > 60.0:
            if not a.allow_drive:
                print(f'  target at r={rng:.0f} y={by:.0f} needs staging but '
                      '--allow-drive is off. Stopping here per operator gate.')
                return 3
            c.move(255, 0, 60, GRIP_CLOSED, speed=90.0)     # tuck
            if abs(bearing) > 0.06:
                fut = c.drv.call_async(DriveRelative.Request(
                    dx=0.0, dy=0.0, dyaw=float(bearing)))
                while not fut.done():
                    c.spin(0.1)
            fwd = max(-0.05, min(0.40, (rng - PICK_R) / 1000.0))
            if abs(fwd) > 0.015:
                fut = c.drv.call_async(DriveRelative.Request(
                    dx=float(fwd), dy=0.0, dyaw=0.0))
                while not fut.done():
                    c.spin(0.1)
                r = fut.result()
                if not r.success:
                    print(f'  staging drive failed: {r.message}')
                    return 3
                rng -= r.actual_dx * 1000.0
            bx, by = rng, 0.0
            print(f'  [stage] target now ({bx:.0f},{by:.0f})')

        rec = DualRecorder(root, idx, f'pick up the {a.object}') \
            if a.record else None
        bx0, by0 = bx, by
        bx, by, wrist_px = c.refine(bx, by, a.prompt, STRATEGIES[a.strategy]['gz'])
        corr_mm = math.hypot(bx - bx0, by - by0) if wrist_px else None
        held = False
        try:
            if c.grasp(bx, by, a.strategy, rec=rec):
                held = c.verify(a.prompt, pick_xy=(bx, by))
        except RuntimeError as e:
            print(f'  aborted: {e}')
        if rec:
            rec.finish(held, {'object': a.object, 'strategy': a.strategy,
                              'pick': [bx, by], 'held': held})
            idx += 1
        if held:
            ok_n += 1
            # only teach the anchor from picks that needed little/no
            # correction: a success after a big correction says where the
            # object WAS, not where the grasp point is
            if wrist_px is not None and (corr_mm is None or corr_mm < 12.0):
                update_anchor(*wrist_px)
            if a.keep and ep == a.episodes - 1:
                print('holding (--keep)')
                break
            p = STRATEGIES[a.strategy]
            c.move(bx + 15.0, by, p['gz'] + 10.0, speed=35.0)
            c.grip(WIDE, secs=1.0)
            c.move(bx + 15.0, by, p['gz'] + 110.0, speed=80.0)
        else:
            miss_n += 1
            c.grip(WIDE, secs=0.8)
            c.move(*OBSERVE, t=GRIP_CLOSED, speed=90.0)

    print(f'\n{ok_n} held / {miss_n} missed'
          + (f' -> {root}' if a.record else ''))
    c.destroy_node()
    rclpy.shutdown()
    return 0 if ok_n else 2


if __name__ == '__main__':
    sys.exit(main())
