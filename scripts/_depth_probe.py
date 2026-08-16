#!/usr/bin/env python3
"""Dump the depth distribution inside the ball's DINO bbox."""
import sys
import time
import numpy as np
import rclpy
sys.path.insert(0, '/scripts')
import cv2
import dino_client
from ball_pick import BallPick, BALL_PROMPT

rclpy.init()
c = BallPick()
assert c.wait_ready() and c.wait_depth()
c.spin(0.5)
cv2.imwrite('/tmp/_dp.png', c.img)
dets = dino_client.detect('/tmp/_dp.png', BALL_PROMPT, confidence=0.25)
if not dets:
    print('no detection')
    sys.exit(2)
x0, y0, x1, y1 = (int(v) for v in dets[0]['box'])
print(f'bbox ({x0},{y0})-({x1},{y1}) score {dets[0]["score"]:.2f}')
d = c.depth[y0:y1, x0:x1].astype(float)
valid = d[(d > 100) & (d < 4000)]
tot = d.size
print(f'{tot} px in bbox, {valid.size} valid ({100*valid.size/tot:.0f}%)')
if valid.size:
    for p in (5, 10, 20, 30, 50, 70, 90):
        print(f'  p{p}: {np.percentile(valid, p):.0f}mm')
    near = valid[valid < np.percentile(valid, 50)]
    print(f'near-cluster median: {np.median(near):.0f}mm')
# also: shrink to the central third of the bbox (pure ball, no edges)
cx0, cy0 = x0 + (x1-x0)//3, y0 + (y1-y0)//3
cx1, cy1 = x1 - (x1-x0)//3, y1 - (y1-y0)//3
dc = c.depth[cy0:cy1, cx0:cx1].astype(float)
vc = dc[(dc > 100) & (dc < 4000)]
print(f'central third: {vc.size}/{dc.size} valid, '
      f'median {np.median(vc):.0f}mm' if vc.size else 'central third: NO depth')
c.destroy_node()
rclpy.shutdown()
