#!/usr/bin/env python3
"""OFFLINE validation of the head-cam 2D + floor-plane fusion (head_scout).

Read-only w.r.t. the robot: subscribes to cameras/detections, publishes ONLY
the /flywheel overlays (so the operator's SUDS Data Flywheel page shows what
the fusion is computing) -- this node has no action publishers at all.

Per iteration: fit the floor plane from the aligned depth's valid band,
ray-cast every head-YOLO detection's floor-contact pixel (box bottom-center)
onto it, convert to arm mm, and track the projected points across frames.
A trustworthy fusion shows per-sock scatter of a few mm; a broken one jumps.

  python3 /scripts/scout_validate.py --secs 20
"""
import argparse
import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

# same empirical cam->arm offsets the pick pipeline uses
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ball_pick import load_cal


class ScoutValidate(Node):
    def __init__(self):
        super().__init__('scout_validate')
        self.br = CvBridge()
        self.img = self.depth = self.K = None
        self.head_dets = self.wrist_img = self.wrist_dets = None
        self.create_subscription(CompressedImage,
                                 '/camera/color/image_raw/compressed',
                                 lambda m: setattr(self, 'img', cv2.imdecode(
                                     np.frombuffer(m.data, np.uint8),
                                     cv2.IMREAD_COLOR)), 2)
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            lambda m: setattr(self, 'depth',
                              self.br.imgmsg_to_cv2(m, 'passthrough')), 2)
        self.create_subscription(
            CameraInfo, '/camera/aligned_depth_to_color/camera_info',
            lambda m: setattr(self, 'K', (m.k[0], m.k[4], m.k[2], m.k[5])), 2)
        self.create_subscription(Detection2DArray, '/head_yolo/detections',
                                 lambda m: setattr(self, 'head_dets', m), 5)
        self.create_subscription(CompressedImage,
                                 '/wrist_cam/image_raw/compressed',
                                 lambda m: setattr(self, 'wrist_img',
                                                   cv2.imdecode(np.frombuffer(
                                                       m.data, np.uint8),
                                                       cv2.IMREAD_COLOR)), 2)
        self.create_subscription(Detection2DArray, '/wrist_yolo/detections',
                                 lambda m: setattr(self, 'wrist_dets', m), 5)
        self.pub_head = self.create_publisher(
            CompressedImage, '/flywheel/head/overlay', 2)
        self.pub_wrist = self.create_publisher(
            CompressedImage, '/flywheel/wrist/overlay', 2)
        self.pub_meta = self.create_publisher(String, '/flywheel/meta', 2)

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def floor_plane(self):
        if self.depth is None or self.K is None:
            return None, 0
        fx, fy, cx, cy = self.K
        H, W = self.depth.shape[:2]
        pts = []
        for v in range(240, min(H, 474), 18):
            for u in range(30, W - 30, 32):
                z = float(self.depth[v, u])
                if 320.0 < z < 2600.0:
                    pts.append(((u - cx) * z / fx, (v - cy) * z / fy, z))
        if len(pts) < 40:
            return None, len(pts)
        P = np.asarray(pts)
        ctr = P.mean(0)
        n = np.linalg.svd(P - ctr)[2][2]
        resid = float(np.abs((P - ctr) @ n).mean())
        return (n, float(n @ ctr), resid), len(pts)

    def loft_of(self, box, plane):
        """Object height above the fitted floor plane, from depth pixels
        inside the box interior (valid band only). None if depth is blind
        there. Positive = toward the camera (above floor)."""
        n, d, _ = plane
        fx, fy, cx, cy = self.K
        x0, y0, x1, y1 = box
        w_, h_ = x1 - x0, y1 - y0
        s = -1.0 if d > 0 else 1.0     # camera origin is above the floor
        resid = []
        for v in np.linspace(y0 + 0.2 * h_, y1 - 0.2 * h_, 7):
            for u in np.linspace(x0 + 0.2 * w_, x1 - 0.2 * w_, 7):
                ui, vi = int(u), int(v)
                if not (0 <= vi < self.depth.shape[0]
                        and 0 <= ui < self.depth.shape[1]):
                    continue
                z = float(self.depth[vi, ui])
                if 150.0 < z < 3000.0:
                    P = np.array([(ui - cx) * z / fx, (vi - cy) * z / fy, z])
                    resid.append(s * (float(n @ P) - d))
        if len(resid) < 6:
            return None
        pos = [r for r in resid if r > 0]
        return float(np.percentile(pos, 75)) if pos else 0.0

    def dets_of(self, msg):
        out = []
        if msg is None:
            return out
        for d in msg.detections:
            cx_ = d.bbox.center.position.x
            cy_ = d.bbox.center.position.y - 80.0
            w_, h_ = d.bbox.size_x, d.bbox.size_y
            s = max((r.hypothesis.score for r in d.results), default=0.0)
            out.append(([cx_ - w_ / 2, cy_ - h_ / 2,
                         cx_ + w_ / 2, cy_ + h_ / 2], s))
        return out

    def publish(self, pub, img, jpeg_q=80):
        m = CompressedImage()
        m.format = 'jpeg'
        m.data = cv2.imencode('.jpg', img,
                              [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])[1].tobytes()
        pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    n = ScoutValidate()
    n.spin(2.0)
    tx, ty = load_cal()
    tracks = []          # [(ax, ay)] clusters across iterations
    samples = {}         # cluster idx -> [(ax, ay, score)]
    t_end = time.time() + a.secs
    it = 0
    while time.time() < t_end:
        n.spin(1.0)
        it += 1
        plane, npts = n.floor_plane()
        if n.img is None or plane is None:
            print(f'it{it}: waiting (img={n.img is not None} '
                  f'plane_pts={npts})')
            continue
        nvec, d, resid = plane
        fx, fy, cx, cy = n.K
        vis = n.img.copy()
        cv2.putText(vis, f'SCOUT VALIDATE plane_pts={npts} '
                    f'resid={resid:.1f}mm', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 220, 255), 2)
        projected = []
        for box, score in n.dets_of(n.head_dets):
            u_, v_ = (box[0] + box[2]) / 2, box[3]
            ray = np.array([(u_ - cx) / fx, (v_ - cy) / fy, 1.0])
            den = float(nvec @ ray)
            if abs(den) < 1e-6:
                continue
            X, _Y, Z = (d / den) * ray
            if not (60.0 < Z < 4000.0):
                continue
            ax, ay = Z + tx, -X + ty
            loft = n.loft_of(box, (nvec, d, resid))
            projected.append((ax, ay, score, loft))
            x0, y0, x1, y1 = (int(t) for t in box)
            cv2.rectangle(vis, (x0, y0), (x1, y1), (80, 220, 80), 2)
            lf = f' loft={loft:.0f}mm' if loft is not None else ' loft=blind'
            cv2.putText(vis, f'{score:.2f} -> ({ax:.0f},{ay:.0f}) '
                        f'r={math.hypot(ax, ay):.0f}{lf}',
                        (x0, max(14, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (80, 220, 80), 2)
            # associate to a track (60mm gate)
            for i, (txx, tyy) in enumerate(tracks):
                if math.hypot(ax - txx, ay - tyy) < 60.0:
                    samples[i].append((ax, ay, score))
                    tracks[i] = (float(np.mean([s[0] for s in samples[i]])),
                                 float(np.mean([s[1] for s in samples[i]])))
                    break
            else:
                tracks.append((ax, ay))
                samples[len(tracks) - 1] = [(ax, ay, score)]
        n.publish(n.pub_head, vis)
        if n.wrist_img is not None:
            wv = n.wrist_img.copy()
            for box, score in n.dets_of(n.wrist_dets):
                x0, y0, x1, y1 = (int(t) for t in box)
                cv2.rectangle(wv, (x0, y0), (x1, y1), (80, 220, 80), 2)
                cv2.putText(wv, f'wrist-yolo {score:.2f}',
                            (x0, max(14, y0 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 220, 80), 2)
            cv2.putText(wv, 'SCOUT VALIDATE (no motion)', (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 220, 255), 2)
            n.publish(n.pub_wrist, wv)
        meta = {'ts': time.time(), 'cam': 'head', 'stage': 'scout-validate',
                'label': 'sock', 'plane_pts': npts,
                'plane_resid_mm': round(resid, 1),
                'targets': [[round(p[0]), round(p[1]), round(p[2], 2)]
                            for p in projected]}
        n.pub_meta.publish(String(data=json.dumps(meta)))
        print(f'it{it}: plane {npts}pts resid {resid:.1f}mm; '
              + ('; '.join(
                  f'({p[0]:.0f},{p[1]:.0f}) s={p[2]:.2f} '
                  f'loft={"?" if p[3] is None else f"{p[3]:.0f}"}'
                  for p in projected) or 'no head dets'))

    print('\n== per-sock projection stability ==')
    for i, pts in samples.items():
        P = np.asarray([(p[0], p[1]) for p in pts])
        m_ = P.mean(0)
        sd = P.std(0)
        print(f'track {i}: n={len(pts)} mean=({m_[0]:.0f},{m_[1]:.0f}) '
              f'r={math.hypot(*m_):.0f} std=({sd[0]:.1f},{sd[1]:.1f})mm '
              f'score~{np.mean([p[2] for p in pts]):.2f}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
