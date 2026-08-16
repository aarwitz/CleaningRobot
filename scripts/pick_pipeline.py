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
import os
import signal
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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from sock_cycle import GRIP_CLOSED, Recorder

ROI = (16, 200, 624, 480)
WIDE = 0.35
PICK_R = 300.0                  # proven grasp radius (vertical-claw posture)
ANCHOR_FILE = '/demos/wrist_anchor.json'

STRATEGIES = {
    # gz: wrist z at close; soft piles sit higher than their contact point
    # secure=3.14 for plush: fur slips and the material is all compressible
    # slack (attempt-1 slip-out at 3.05); band compliance protects the servo
    'sweep': dict(gz=-172.0, approach_dy=60.0, overshoot=12.0,
                  close_to=2.95, close_start=0.25, secure=3.14),
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
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def hop(self, dx=0.0, dyaw=0.0, speed=0.14, yaw_speed=0.5):
        """Open-loop timed /cmd_vel burst for staging — NO encoder reads.

        DriveRelative's encoder read-between-bursts has hung the 0x34 board
        off the I2C bus twice this session (needs a power cycle each time).
        Staging hops are re-verified by the vision loop and the wrist refine
        absorbs the ±15% open-loop error, so pure-write driving is the
        reliable choice. Calibration from drive_open_loop.py (0.74 delivery)."""
        def burst(vx, wz, secs):
            m = Twist()
            m.linear.x, m.angular.z = float(vx), float(wz)
            t0 = time.time()
            while time.time() - t0 < secs:
                self.cmd.publish(m)
                self.spin(1.0 / 20.0)
            z = Twist()
            for _ in range(8):
                self.cmd.publish(z)
                self.spin(1.0 / 20.0)
        if abs(dyaw) > 0.02:
            burst(0.0, yaw_speed * (1 if dyaw > 0 else -1),
                  abs(dyaw) / yaw_speed)
            self.spin(0.4)
        if abs(dx) > 0.01:
            burst(speed * (1 if dx > 0 else -1), 0.0,
                  abs(dx) / (speed * 0.74))
            self.spin(0.4)

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

    def is_claw_det(self, box, whitelist=None):
        """The claw's own fingers get detected as objects (seen: 'white
        fluffy rabbit toy' 0.39 on the black finger). The fingers live in a
        fixed region of the wrist image AND are near-black; both together =
        reject. A DARK floor object near the claw matches both tests too
        (2026-08-11: black sock at (157,384) rejected every scan while the
        operator could pick it by hand) -- `whitelist` carries pixel centers
        that a parallax probe proved are NOT the claw."""
        if self.wrist is None:
            return False
        u = (box[0] + box[2]) / 2
        v = (box[1] + box[3]) / 2
        if whitelist and min(math.hypot(u - wu, v - wv)
                             for wu, wv in whitelist) < 40.0:
            return False
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

    def distractor_filter(self, dets, img_path, min_area=6000.0):
        """Confidence alone cannot separate junk from socks (measured
        2026-08-11: creamer cup 0.42 > white sock 0.38 > black sock 0.33).
        Two signals that DO separate: real in-reach objects subtend >>6k px^2
        at hover (creamer det was 3k), and a counter-prompt claims metallic
        junk more strongly than 'sock' does."""
        kept = []
        for d_ in dets:
            b = d_['box']
            if (b[2] - b[0]) * (b[3] - b[1]) < min_area:
                print(f'  [wrist] rejecting tiny det '
                      f'({(b[2]-b[0])*(b[3]-b[1]):.0f} px^2)')
                continue
            kept.append(d_)
        if not kept:
            return kept
        try:
            neg = dino_client.detect(img_path, 'metal cup. mug. jar. bottle. '
                                     'plastic bag.', confidence=0.25)
        except Exception:
            return kept          # counter-check unavailable: do not block
        def iou(a, b):
            ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
            iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
            inter = ix * iy
            ua = ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter)
            return inter / ua if ua > 0 else 0.0
        out = []
        for d_ in kept:
            cf = d_.get('confidence') or d_.get('score') or 0.0
            hit = next((n_ for n_ in neg if iou(d_['box'], n_['box']) > 0.5 and
                        (n_.get('confidence') or n_.get('score') or 0) > cf),
                       None)
            if hit:
                print(f"  [wrist] rejecting det: counter-prompt "
                      f"'{hit.get('label')}' "
                      f"{(hit.get('confidence') or hit.get('score')):.2f} "
                      f"beats sock {cf:.2f}")
                continue
            out.append(d_)
        return out

    # ── refine: wrist cam from the pre-grasp hover ──────────────────────────
    def refine(self, bx, by, prompt, gz, cap=65.0, lock_xy=None,
               claw_whitelist=None):
        """Hover above the coarse target, detect in the WRIST frame, correct
        (bx, by) by the offset from the grasp anchor. Returns possibly
        corrected (bx, by); no-ops gracefully without wrist frames/anchor."""
        pre = self.wrist.copy() if self.wrist is not None else None
        self.move(bx, by, gz + 130.0, t=WIDE, speed=90.0, grip_ramp=True)
        self.spin(0.9)
        if self.wrist is None:
            print('  [refine] no wrist frames -- skipping')
            return bx, by, None
        if pre is not None and \
                float(np.mean(cv2.absdiff(pre, self.wrist))) < 0.5:
            # frames arriving at full rate but content identical across an
            # arm move = stale stream (seen 2026-08-02: refine chased
            # corrections computed on outdated viewpoints). Never correct on
            # a frame that predates the motion.
            print('  [refine] wrist stream STALE across the hover move -- '
                  'refusing to correct')
            return bx, by, None
        self.ensure_light()
        cv2.imwrite('/tmp/_wrist.png', self.wrist)
        try:
            dets = dino_client.detect('/tmp/_wrist.png', prompt,
                                      confidence=0.30)
            dets = self.distractor_filter(dets, '/tmp/_wrist.png')
            rejected = [d_ for d_ in dets
                        if self.is_claw_det(d_['box'], claw_whitelist)]
            dets = [d_ for d_ in dets if d_ not in rejected]
        except Exception as e:
            # tunnel-down must not read as "object not there"
            print(f'  [refine] DINO unreachable: {e}')
            dets, rejected = [], []
        if not dets and rejected and claw_whitelist is None:
            # Everything we saw was dark-in-the-claw-zone. The fingers ride
            # WITH the camera (fixed pixels across arm moves); a floor object
            # shifts. Jog the hover and see which one this is.
            old_px = [((d_['box'][0] + d_['box'][2]) / 2,
                       (d_['box'][1] + d_['box'][3]) / 2) for d_ in rejected]
            print('  [refine] only claw-zone dark detections -- parallax '
                  'probe (+40mm y jog)')
            self.move(bx, by + 40.0, gz + 130.0, t=WIDE, speed=90.0)
            self.spin(0.9)
            mobile = []
            if self.wrist is not None:
                cv2.imwrite('/tmp/_wrist.png', self.wrist)
                try:
                    nd = dino_client.detect('/tmp/_wrist.png', prompt,
                                            confidence=0.30)
                except Exception as e:
                    print(f'  [refine] DINO unreachable in probe: {e}')
                    nd = []
                new_px = [((d_['box'][0] + d_['box'][2]) / 2,
                           (d_['box'][1] + d_['box'][3]) / 2) for d_ in nd]
                # a finger re-detects at (nearly) the same pixel after the
                # jog; a floor object's old pixel is left vacant
                for ou, ov in old_px:
                    if min((math.hypot(nu - ou, nv - ov)
                            for nu, nv in new_px), default=1e9) > 25.0:
                        mobile.append((ou, ov))
            if mobile:
                print(f'  [refine] parallax: {len(mobile)} det(s) SHIFTED -> '
                      'real floor object, not the claw')
                return self.refine(bx, by, prompt, gz, cap=cap,
                                   lock_xy=lock_xy or (bx, by),
                                   claw_whitelist=mobile)
            print('  [refine] parallax: detections static -> genuinely the '
                  'claw')
        if not dets:
            print('  [refine] no wrist detection -- keeping coarse target')
            self.publish_flywheel(self.wrist,
                                  {'stage': 'refine (wrist cam): no det',
                                   'label': prompt.split('.')[0]},
                                  cam='wrist')
            return bx, by, None
        anc = load_anchor()

        # the wrist cam rides the base servo: its px axes are the CALIBRATION
        # pose's axes rotated by the current base yaw. The Jacobian was
        # measured at (300,0) (yaw~0); apply R(yaw) or off-axis corrections
        # steer sideways (observed: circular multi-pass chase at y~+90,
        # yaw~20 deg, while every near-axis pick converged fine)
        th = math.atan2(by, bx)
        cth, sth = math.cos(th), math.sin(th)

        def corr_of(box_):
            """px offset from anchor -> (dx,dy) arm-mm correction."""
            u_, v_ = (box_[0] + box_[2]) / 2, (box_[1] + box_[3]) / 2
            du_, dv_ = u_ - anc['u'], v_ - anc['v']
            if 'Jinv' in anc:
                Ji = anc['Jinv']
                lx, ly = (Ji[0][0] * du_ + Ji[0][1] * dv_,
                          Ji[1][0] * du_ + Ji[1][1] * dv_)
            else:
                lx, ly = dv_ * anc['mm_per_px'], -du_ * anc['mm_per_px']
            return (cth * lx - sth * ly, sth * lx + cth * ly)

        if anc is not None and len(dets) > 1:
            # cluttered scene: several valid objects in view. Pixel-space
            # stickiness fails because the view moves between passes --
            # lock the target in ARM coordinates: keep the detection whose
            # implied arm position is nearest the locked target (observed
            # without this: 4-pass oscillation chasing different socks)
            if lock_xy is not None:
                dets.sort(key=lambda d_: math.hypot(
                    bx + corr_of(d_['box'])[0] - lock_xy[0],
                    by + corr_of(d_['box'])[1] - lock_xy[1]))
            else:
                dets.sort(key=lambda d_: math.hypot(
                    (d_['box'][0] + d_['box'][2]) / 2 - anc['u'],
                    (d_['box'][1] + d_['box'][3]) / 2 - anc['v']))
        wbox = dets[0]['box']
        wu, wv = (wbox[0] + wbox[2]) / 2, (wbox[1] + wbox[3]) / 2
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
        # wrist cam rigid to EE: px offset -> arm offset. Use the measured
        # 2x2 Jacobian when calibrated (--calibrate-wrist); the hand-assumed
        # axis mapping had the x sign backwards and corrected a far object
        # NEARER (attempt-5 short pick, operator-diagnosed).
        dx_mm, dy_mm = corr_of(wbox)
        n = math.hypot(dx_mm, dy_mm)
        if n > cap:
            # log the raw magnitude: a consistently-capped correction is a
            # SYSTEMATIC staging bias (open-loop yaw), not detection noise
            print(f'  [refine] raw correction {n:.0f}mm capped at {cap:.0f}')
            dx_mm, dy_mm = dx_mm * cap / n, dy_mm * cap / n
        print(f'  [refine] wrist px ({wu:.0f},{wv:.0f}) vs anchor '
              f'({anc["u"]:.0f},{anc["v"]:.0f}) -> correct '
              f'({dx_mm:+.0f},{dy_mm:+.0f})mm')
        # remember the OTHER detections' implied arm positions so the grasp
        # can tell when a neighbor sits inside the sweep corridor
        self.neighbors = []
        for d_ in dets[1:]:
            ndx, ndy = corr_of(d_['box'])
            self.neighbors.append((bx + ndx, by + ndy))
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
        # neighbor-aware sweep: the full +y approach scoops a second object
        # that sits in the corridor (audit 2026-08-02: three double-grasp
        # episodes) -- shorten the runway so the jaws close on ONE object
        adx = p['approach_dy']
        for nx, ny in getattr(self, 'neighbors', []):
            if abs(nx - bx) < 55.0 and 15.0 < ny - by < adx + 40.0:
                adx = 30.0
                print(f'  [grasp] neighbor at ({nx:.0f},{ny:.0f}) in sweep '
                      f'corridor -> short sweep (approach_dy {adx:.0f})')
                break
        self.move(bx, by + adx, gz + 110.0, t=WIDE, speed=90.0,
                  grip_ramp=True, rec=rec, phase='approach')
        self.spin(0.8)
        if self.pose()[3] > 1.25:
            print('  gripper failed to open')
            return False
        self.move(bx, by + adx, gz, speed=45.0, settle=False,
                  rec=rec, phase='descend')
        self.sweep_in(bx, by, gz, adx, p['overshoot'],
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
    def verify(self, prompt, pick_xy=None, rec=None):
        """pick_xy restricts the head floor-check to the pick's own bearing:
        other same-class objects in the scene (a second plush) must not
        false-flag 'still on the floor'. rec: save the exact frames the
        verdict was computed from into the episode (audit trail)."""
        held_head = held_wrist = None
        self._wrist_overwhelming = False
        if rec is not None:
            cv2.imwrite(str(rec.dir / 'verify_head.jpg'), self.img)
            if self.wrist is not None:
                cv2.imwrite(str(rec.dir / 'verify_wrist.jpg'), self.wrist)
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
                # From the HIGH verify pose the discrimination is easy: a
                # held object is centimeters from the wrist lens (huge bbox);
                # a miss shows distant floor (small/no detection). Area is
                # the signal. Do NOT apply the claw dark-filter here: a held
                # DARK sock in the claw zone is exactly what it rejects
                # (observed false-miss on the navy sock, 2026-08-02).
                areas = [(d_['box'][2] - d_['box'][0]) *
                         (d_['box'][3] - d_['box'][1]) for d_ in vw]
                # thresholds re-derived from the 2026-08-02 audit: from the
                # HIGH pose a sock on the FLOOR still subtends ~15-20k px^2
                # (ep_0075 false held at >18k); a truly held sock is
                # centimeters from the lens and fills >100k
                held_wrist = any(ar > 60000 for ar in areas)
                # an object FILLING the frame is unambiguous -- it overrides
                # the head floor-check, which cannot tell 'my target is
                # still down there' from 'a DIFFERENT object is down there'
                # in multi-object scenes (ep_0069 false miss)
                self._wrist_overwhelming = any(ar > 120000 for ar in areas)
            except Exception:
                pass
        # both cameras must agree when both have an opinion; an
        # overwhelming wrist hold (frame-filling object) wins outright
        votes = [x for x in (held_head, held_wrist) if x is not None]
        held = (bool(votes) and all(votes)) or \
            getattr(self, '_wrist_overwhelming', False)
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


def calibrate_wrist(c, a):
    """Empirical wrist px->mm Jacobian: from the pre-grasp hover, jog the EE
    a known +/-25mm in arm x then y, DINO-track the object's pixel each time,
    and solve [du,dv] = J @ [dx,dy]. Saves J^-1 into the anchor file. Kills
    the assumed-axes sign errors for good."""
    bx, by, why, _ = c.coarse(a.prompt)
    if why != 'ok':
        # head cam is blind inside the arm's workspace; the jogs only need
        # SOME object in the wrist view -- hover at nominal and use whatever
        # the wrist sees
        print(f'  [cal] head cam cannot localize ({why}) -- wrist-only '
              'from nominal hover')
        bx, by = 300.0, 0.0
    gz = STRATEGIES[a.strategy]['gz']
    hover = gz + 130.0

    prev = [None]

    def see(tag):
        c.spin(0.8)
        cv2.imwrite('/tmp/_calw.png', c.wrist)
        try:
            dets = dino_client.detect('/tmp/_calw.png', a.prompt,
                                      confidence=0.30)
            dets = [d_ for d_ in dets if not c.is_claw_det(d_['box'])]
        except Exception as e:
            print(f'  {tag}: detect failed {e}')
            return None
        if not dets:
            print(f'  {tag}: no detection')
            return None
        # clutter: must TRACK ONE OBJECT across jogs, or the columns mix
        # different socks' positions and the solved J is garbage
        if prev[0] is not None:
            dets.sort(key=lambda d_: math.hypot(
                (d_['box'][0]+d_['box'][2])/2 - prev[0][0],
                (d_['box'][1]+d_['box'][3])/2 - prev[0][1]))
        b = dets[0]['box']
        p = ((b[0]+b[2])/2, (b[1]+b[3])/2)
        if prev[0] is None:
            prev[0] = p
        print(f'  {tag}: px ({p[0]:.0f},{p[1]:.0f})')
        return p

    c.move(bx, by, hover, t=WIDE, speed=90.0, grip_ramp=True)
    p0 = see('center')
    if p0 is None:
        return 2
    D = 40.0
    # jog -x (retract): +x walks the claw OVER the object and occludes it
    c.move(bx - D, by, hover, speed=40.0)
    px_ = see('-x')
    c.move(bx, by, hover, speed=40.0)
    c.move(bx, by + D, hover, speed=40.0)
    py_ = see('+y')
    c.move(bx, by, hover, speed=40.0)
    if px_ is None or py_ is None:
        return 2
    # object pixel moves OPPOSITE to the EE, so J columns are negated deltas
    # x jog is NEGATIVE D: object px delta already has the sign of -x, so
    # dividing by +D gives the +x column directly
    ju = ((px_[0]-p0[0])/D, -(py_[0]-p0[0])/D)
    jv = ((px_[1]-p0[1])/D, -(py_[1]-p0[1])/D)
    det = ju[0]*jv[1] - ju[1]*jv[0]
    if abs(det) < 0.3:
        # near-singular = one jog produced almost no pixel motion (tracked
        # the wrong object, or the axis is unobservable from this pose);
        # its inverse would emit wild corrections -- refuse to save
        print(f'near-singular Jacobian (det {det:.3f}); NOT saving')
        return 2
    Jinv = [[jv[1]/det, -ju[1]/det], [-jv[0]/det, ju[0]/det]]
    anc = load_anchor() or {'u': p0[0], 'v': p0[1], 'n': 0, 'mm_per_px': 0.55}
    anc['Jinv'] = Jinv
    anc['J'] = [[ju[0], ju[1]], [jv[0], jv[1]]]
    Path(ANCHOR_FILE).write_text(json.dumps(anc))
    print(f'Jacobian solved: J=[[{ju[0]:.2f},{ju[1]:.2f}],'
          f'[{jv[0]:.2f},{jv[1]:.2f}]] px/mm -> saved with anchor')
    return 0


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


def self_anchor(c, a):
    """Autonomously re-learn the grasp anchor after a camera-mount change.

    The anchor ('object at this wrist pixel == object in the jaws') can be
    measured by the robot itself: attempt grasps around the current best
    guess with small spiral offsets; on the FIRST hold, carry the object to
    the standard hover, release it, and detect where it lands. The landing
    pixel is the grasp point as seen by the (new) camera orientation --
    a gold sample with no human in the loop."""
    gz = STRATEGIES[a.strategy]['gz']
    offsets = [(0, 0), (15, 0), (-15, 0), (0, 15), (0, -15),
               (15, 15), (-15, -15), (20, -20)]
    for i, (ox, oy) in enumerate(offsets):
        print(f'\n── self-anchor attempt {i+1}/{len(offsets)} '
              f'offset ({ox:+.0f},{oy:+.0f}) ──')
        bx, by, wrist_px = c.refine(300.0, 0.0, a.prompt, gz, cap=140.0)
        if wrist_px is None:
            for sx, sy in ((260.0, 70.0), (320.0, -70.0),
                           (250.0, -70.0), (340.0, 60.0)):
                bx, by, wrist_px = c.refine(sx, sy, a.prompt, gz, cap=140.0)
                if wrist_px is not None:
                    break
        if wrist_px is None:
            print('  no object visible from any hover; stopping')
            return 2
        lock = (bx, by)
        for _ in range(2):
            p_prev = (bx, by)
            bx, by, wp = c.refine(bx, by, a.prompt, gz, lock_xy=lock)
            if wp is None:
                bx, by = p_prev
                break
            lock = (bx, by)
            if math.hypot(bx - p_prev[0], by - p_prev[1]) < 15.0:
                break
        try:
            c.grasp(bx + ox, by + oy, a.strategy)
        except RuntimeError as e:
            print(f'  aborted: {e}')
            continue
        held = False
        if c.wrist is not None:
            cv2.imwrite('/tmp/_sa.png', c.wrist)
            try:
                vw = dino_client.detect('/tmp/_sa.png', a.prompt,
                                        confidence=0.30)
                vw = [d_ for d_ in vw if not c.is_claw_det(d_['box'])]
                held = any((d_['box'][2] - d_['box'][0]) *
                           (d_['box'][3] - d_['box'][1]) > 18000
                           for d_ in vw)
            except Exception:
                pass
        print(f'  held={held}')
        if not held:
            c.grip(WIDE, secs=0.8)
            continue
        # carry to the standard hover, release, and observe the landing
        c.move(300.0, 0.0, gz + 130.0, speed=50.0)
        c.spin(0.5)
        c.grip(WIDE, secs=1.0)
        c.spin(1.5)
        cv2.imwrite('/tmp/_sa_land.png', c.wrist)
        try:
            dets = dino_client.detect('/tmp/_sa_land.png', a.prompt,
                                      confidence=0.30)
            dets = [d_ for d_ in dets if not c.is_claw_det(d_['box'])]
        except Exception as e:
            print(f'  landing detect failed: {e}')
            return 2
        if not dets:
            print('  released object not visible from hover; cannot anchor')
            return 2
        # the dropped object is directly under the lens: take the LARGEST box
        dets.sort(key=lambda d_: -(d_['box'][2] - d_['box'][0]) *
                                  (d_['box'][3] - d_['box'][1]))
        b = dets[0]['box']
        u, v = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        anc = load_anchor() or {'mm_per_px': 0.55}
        anc.update({'u': u, 'v': v, 'n': 3})
        Path(ANCHOR_FILE).write_text(json.dumps(anc))
        print(f'  ANCHOR LEARNED: ({u:.0f},{v:.0f}) from self-drop '
              f'(offset that held: ({ox:+.0f},{oy:+.0f}))')
        return 0
    print('no hold anywhere in the offset spiral; anchor not learned')
    return 2


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
    ap.add_argument('--calibrate-wrist', action='store_true',
                    help='solve the wrist px->mm Jacobian empirically: hover '
                         'over the object, jog the EE +/-25mm in x and y, '
                         'track how its pixel moves')
    ap.add_argument('--teach', action='store_true',
                    help='human-in-the-loop pick-pose demos: operator '
                         'positions the open claw, system closes/lifts/'
                         'verifies and learns anchor + grasp z')
    ap.add_argument('--self-anchor', action='store_true',
                    help='autonomously re-learn the grasp anchor: attempt '
                         'grasps with small spiral offsets; on the first '
                         'hold, release the object from the hover and take '
                         'its landing pixel as the anchor (no human demos)')
    ap.add_argument('--wrist-only', action='store_true',
                    help='skip head-cam coarse + staging: hover at a nominal '
                         'in-reach spot and localize purely with the wrist '
                         'cam. For arm-only sessions (base powered off) where '
                         'the object is in reach but below the head cam\'s '
                         'depth-valid band.')
    ap.add_argument('--gz', type=float, default=None,
                    help='override grasp z (beats strategy default and bank)')
    ap.add_argument('--drop-at', type=str, default=None,
                    help='"x,y" arm coords to release at instead of the pick '
                         'spot')
    ap.add_argument('--out', type=str, default='/demos/picks')
    a = ap.parse_args()

    rclpy.init()
    c = PickPipeline()

    def _halt_trap(signum, _frm):
        # A killed batch script must NEVER leave the arm parked at its last
        # commanded pose: at grasp depth the servos hunt against the floor
        # (observed 2026-08-15, HANDOFF §5.3). Lift to the tucked pose
        # (closed grip -- the in-run tuck convention; nothing held gets
        # flung), then hard-exit: normal cleanup can hang mid-spin.
        # `robot halt` also SIGTERMs us and re-sends the same lift as
        # belt-and-braces, so a double send is expected and harmless.
        print(f'\n[halt-trap] signal {signum}: lifting to tucked safe pose',
              flush=True)
        try:
            c.send(255, 0, 60, GRIP_CLOSED)
            time.sleep(0.5)     # let the publish flush before exiting
        finally:
            os._exit(65)

    signal.signal(signal.SIGTERM, _halt_trap)
    signal.signal(signal.SIGINT, _halt_trap)

    if not c.wait_ready() or not c.wait_depth():
        print('ABORT: teleop/camera/depth not ready')
        return 1
    root = Path(a.out)
    if a.record:
        root.mkdir(parents=True, exist_ok=True)
    if a.calibrate_wrist:
        rc = calibrate_wrist(c, a)
        c.destroy_node()
        rclpy.shutdown()
        return rc
    if a.self_anchor:
        rc = self_anchor(c, a)
        c.destroy_node()
        rclpy.shutdown()
        return rc
    if a.teach:
        teach(c, a, root)
        c.destroy_node()
        rclpy.shutdown()
        return 0
    idx = 1 + max([int(p.name[3:]) for p in root.glob('ep_*')] or [-1]) \
        if a.record else 0
    STRATEGIES[a.strategy]['gz'] = taught_gz(a.object,
                                             STRATEGIES[a.strategy]['gz'])
    if a.gz is not None:
        STRATEGIES[a.strategy]['gz'] = a.gz
        print(f'  [gz] operator override {a.gz:.0f}')

    ok_n = miss_n = 0
    for ep in range(a.episodes):
        print(f'\n── {a.object} attempt {ep+1}/{a.episodes} ──')
        if a.wrist_only:
            # the head cam cannot depth-localize inside the arm's workspace
            # (blind <300mm, box hits frame bottom); the wrist cam owns
            # anything already in reach
            bx, by, why, u0 = 300.0, 0.0, 'ok', None
            print('  [wrist-only] nominal hover (300,0); wrist cam localizes')
        else:
            bx, by, why, u0 = c.coarse(a.prompt)
        if why == 'blind' and a.allow_drive:
            # object inside the depth blind zone: tucked-arm backup, coarse
            # again from depth-valid range (the drive-back happens in the
            # normal staging block below)
            print('  [coarse] blind -> tucked open-loop backup 0.18m')
            c.move(255, 0, 60, GRIP_CLOSED, speed=90.0)
            c.hop(dx=-0.18)
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
            fwd = max(-0.05, min(0.40, (rng - PICK_R) / 1000.0))
            c.hop(dx=fwd, dyaw=bearing if abs(bearing) > 0.06 else 0.0)
            # open-loop: assume nominal; the wrist refine absorbs the error
            bx, by = rng - fwd * 1000.0, 0.0
            print(f'  [stage] open-loop hop done; nominal target '
                  f'({bx:.0f},{by:.0f})')

        rec = DualRecorder(root, idx, f'pick up the {a.object}') \
            if a.record else None
        bx0, by0 = bx, by
        bx, by, wrist_px = c.refine(bx, by, a.prompt, STRATEGIES[a.strategy]['gz'],
                                    cap=140.0 if a.wrist_only else 65.0)
        if a.wrist_only:
            if wrist_px is None:
                # scan other in-reach hover spots: dropped objects scatter
                # beyond the single nominal hover's wrist FOV
                for sx, sy in ((260.0, 70.0), (320.0, -70.0), (250.0, -70.0),
                               (340.0, 60.0)):
                    print(f'  [wrist-only] scanning hover ({sx:.0f},{sy:.0f})')
                    bx, by, wrist_px = c.refine(sx, sy, a.prompt,
                                                STRATEGIES[a.strategy]['gz'])
                    if wrist_px is not None:
                        break
            if wrist_px is None:
                print('  [wrist-only] wrist cannot see the object -- skipping '
                      '(no blind grasp at the nominal spot)')
                miss_n += 1
                if rec:
                    rec.finish(False, {'object': a.object, 'note': 'no wrist det'})
                    idx += 1
                continue
            # converge on the LOCKED target: the initial full-cap pass fixed
            # which physical object we are picking (arm coords); later passes
            # only trim residual error on that same object
            lock = (bx, by)
            for _ in range(3):
                px_prev = (bx, by)
                bx, by, wrist_px = c.refine(bx, by, a.prompt,
                                            STRATEGIES[a.strategy]['gz'],
                                            lock_xy=lock)
                if wrist_px is None:
                    bx, by = px_prev       # keep last good estimate
                    wrist_px = (0, 0)      # target was seen; do not abort
                    break
                lock = (bx, by)
                if math.hypot(bx - px_prev[0], by - px_prev[1]) < 15.0:
                    break
            # floor-grasp reach gate: beyond r~340 the jaw tips rise off the
            # floor (posture-signed drop) and send() silently CLAMPS the
            # command -- the pick shorts every time. Skip; it needs a base
            # move, not more arm.
            rng_ = math.hypot(bx, by)
            if rng_ > 340.0:
                print(f'  [wrist-only] target r={rng_:.0f} is beyond floor-'
                      'grasp reach (~340)')
                near = [(nx, ny) for nx, ny in getattr(c, 'neighbors', [])
                        if math.hypot(nx, ny) <= 340.0]
                if near:
                    bx, by = min(near, key=lambda p_: math.hypot(*p_))
                    print(f'  [wrist-only] retargeting in-reach neighbor '
                          f'({bx:.0f},{by:.0f})')
                    bx, by, wrist_px = c.refine(bx, by, a.prompt,
                                                STRATEGIES[a.strategy]['gz'],
                                                lock_xy=(bx, by))
                if wrist_px is None or math.hypot(bx, by) > 340.0:
                    print('  [wrist-only] nothing in reach -- skipping; '
                          'needs a base move')
                    miss_n += 1
                    if rec:
                        rec.finish(False, {'object': a.object,
                                           'note': f'out of reach r={rng_:.0f}'})
                        idx += 1
                    continue
        corr_mm = math.hypot(bx - bx0, by - by0) if wrist_px else None
        held = False
        try:
            if c.grasp(bx, by, a.strategy, rec=rec):
                held = c.verify(a.prompt, pick_xy=(bx, by), rec=rec)
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
            if a.drop_at:
                px_, py_ = (float(v) for v in a.drop_at.split(','))
            else:
                px_, py_ = bx + 15.0, by
            c.move(px_, py_, p['gz'] + 110.0, speed=60.0)
            c.move(px_, py_, p['gz'] + 10.0, speed=35.0)
            c.grip(WIDE, secs=1.0)
            c.move(px_, py_, p['gz'] + 110.0, speed=80.0)
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
