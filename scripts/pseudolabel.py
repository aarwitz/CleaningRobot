#!/usr/bin/env python3
"""Pseudo-label a YOLO training set from recorded episodes (both cameras).

The flywheel's distillation step: GroundingDINO (the designated open-vocab
pseudo-labeler, SSH tunnel :8002) labels sampled episode frames; the same
heuristics that keep the live pipeline honest gate the labels:

  - counter-prompt arbitration  (the sock-filled plastic bag, the creamer)
  - min-area                    (tiny junk dets)
  - per-episode STATIC-BOX filter on wrist frames: the claw rides the
    camera, so a box whose center barely moves across an episode's samples
    is the claw. The box is dropped but the IMAGE IS KEPT -- an unlabeled
    claw in a training image is a hard negative, which is precisely the
    signal socks2 lacks (it was trained head-cam only and scores the claw
    as a sock from the wrist).

Output: YOLO-format dataset (images/, labels/, dataset.yaml, provenance
jsonl) ready for an ultralytics fine-tune on the GPU box.

  python3 /scripts/pseudolabel.py --demos /demos/picks --out /demos/pseudo_v1 \
      [--per-ep 6] [--limit-eps 8] [--min-score 0.35]
"""
import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dino_client

PROMPT = 'sock. clothing item.'
COUNTER = 'metal cup. mug. jar. bottle. plastic bag.'
MIN_AREA = 5000.0
STATIC_PX = 18.0          # per-episode center movement below this = claw


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = ((a[2] - a[0]) * (a[3] - a[1])
          + (b[2] - b[0]) * (b[3] - b[1]) - inter)
    return inter / ua if ua > 0 else 0.0


def detect(path, min_score, img_wh=(640, 480)):
    """DINO + counter-prompt + min-area + mega-box gates.
    Returns [(box, score)]."""
    W, H = img_wh
    dets = dino_client.detect(str(path), PROMPT, confidence=min_score)
    try:
        neg = dino_client.detect(str(path), COUNTER, confidence=0.25)
    except Exception:
        neg = []
    out = []
    for d in dets:
        b, s = d['box'], (d.get('confidence') or d.get('score') or 0.0)
        bw, bh = b[2] - b[0], b[3] - b[1]
        if bw * bh < MIN_AREA:
            continue
        # mega-box gate: DINO's frame-wide claw/floor strips are label
        # poison (spot-checked 2026-08-16: 638px-wide "sock" strips)
        if bw > 0.85 * W or bh > 0.85 * H or bw * bh > 0.55 * W * H:
            continue
        hit = next((n for n in neg if iou(b, n['box']) > 0.5 and
                    (n.get('confidence') or n.get('score') or 0) > s), None)
        if hit:
            continue
        out.append(([float(t) for t in b], float(s)))
    return out


def sample_frames(ep, cam, per_ep):
    d = ep / ('frames' if cam == 'head' else 'frames_wrist')
    if not d.is_dir():
        return []
    fr = sorted(d.glob('*.jpg'))
    if len(fr) <= per_ep:
        return fr
    # spread across the episode: hover, refine, grasp, verify all appear
    idx = np.linspace(0, len(fr) - 1, per_ep).astype(int)
    return [fr[i] for i in sorted(set(idx))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demos', default='/demos/picks')
    ap.add_argument('--out', default='/demos/pseudo_v1')
    ap.add_argument('--per-ep', type=int, default=6)
    ap.add_argument('--limit-eps', type=int, default=0,
                    help='0 = all episodes')
    ap.add_argument('--min-score', type=float, default=0.35)
    ap.add_argument('--val-frac', type=float, default=0.15)
    a = ap.parse_args()

    eps = sorted(Path(a.demos).glob('ep_*'))
    if a.limit_eps:
        eps = eps[:a.limit_eps]
    out = Path(a.out)
    for sub in ('images/train', 'images/val', 'labels/train', 'labels/val'):
        (out / sub).mkdir(parents=True, exist_ok=True)

    rng = random.Random(0)
    stats = dict(imgs=0, boxes=0, static_dropped=0, empty=0, dino_fail=0)
    prov = open(out / 'provenance.jsonl', 'w')
    t0 = time.time()

    for ep in eps:
        # CLASS HYGIENE: socks2 is a SOCK detector. Episodes of other
        # objects (rabbit, fish, ball) contribute as PURE NEGATIVES: image
        # kept, all boxes dropped -- an unlabeled plush teaches plush!=sock.
        # Ambiguity guard: if a strong (>0.5) sock-like det appears in a
        # non-sock episode frame, skip that frame (might be a real sock in
        # the scene; an unlabeled real sock would be a harmful false
        # negative).
        try:
            ep_obj = json.loads((ep / 'meta.json').read_text()).get('object')
        except Exception:
            ep_obj = None
        is_sock_ep = (ep_obj == 'sock')
        for cam in ('head', 'wrist'):
            frames = sample_frames(ep, cam, a.per_ep)
            if not frames:
                continue
            per_frame = []
            for f in frames:
                try:
                    per_frame.append((f, detect(f, a.min_score)))
                except Exception as e:
                    stats['dino_fail'] += 1
                    print(f'  [dino] {f.name}: {e}')
                    per_frame.append((f, []))
            # per-episode static clusters (claw): centers that persist with
            # tiny movement across most sampled wrist frames
            static_centers = []
            if cam == 'wrist':
                centers = []
                for _f, dets in per_frame:
                    centers += [(((b[0] + b[2]) / 2, (b[1] + b[3]) / 2))
                                for b, _s in dets]
                used = [False] * len(centers)
                for i, c in enumerate(centers):
                    if used[i]:
                        continue
                    cl = [c]
                    for j in range(i + 1, len(centers)):
                        if not used[j] and math.hypot(
                                centers[j][0] - c[0],
                                centers[j][1] - c[1]) < STATIC_PX:
                            used[j] = True
                            cl.append(centers[j])
                    if len(cl) >= max(3, int(0.6 * len(per_frame))):
                        static_centers.append(
                            (float(np.mean([p[0] for p in cl])),
                             float(np.mean([p[1] for p in cl]))))
            for f, dets in per_frame:
                kept = []
                for b, s in dets:
                    cx_, cy_ = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
                    if any(math.hypot(cx_ - u, cy_ - v) < STATIC_PX
                           for u, v in static_centers):
                        stats['static_dropped'] += 1
                        continue
                    kept.append((b, s))
                if not is_sock_ep:
                    if any(s > 0.5 for _b, s in kept):
                        stats['ambiguous_skipped'] = \
                            stats.get('ambiguous_skipped', 0) + 1
                        continue
                    kept = []          # negative sample: image, no boxes
                img = cv2.imread(str(f))
                if img is None:
                    continue
                H, W = img.shape[:2]
                split = 'val' if rng.random() < a.val_frac else 'train'
                stem = f'{ep.name}_{cam}_{f.stem}'
                cv2.imwrite(str(out / f'images/{split}/{stem}.jpg'), img)
                with open(out / f'labels/{split}/{stem}.txt', 'w') as lf:
                    for b, s in kept:
                        cxn = (b[0] + b[2]) / 2 / W
                        cyn = (b[1] + b[3]) / 2 / H
                        wn, hn = (b[2] - b[0]) / W, (b[3] - b[1]) / H
                        lf.write(f'0 {cxn:.6f} {cyn:.6f} {wn:.6f} {hn:.6f}\n')
                stats['imgs'] += 1
                stats['boxes'] += len(kept)
                stats['empty'] += not kept
                prov.write(json.dumps(
                    {'img': f'{stem}.jpg', 'ep': ep.name, 'cam': cam,
                     'frame': f.name, 'split': split,
                     'boxes': [[round(t) for t in b] + [round(s, 3)]
                               for b, s in kept]}) + '\n')
        print(f'{ep.name} done ({stats["imgs"]} imgs, {stats["boxes"]} boxes, '
              f'{time.time() - t0:.0f}s)')
    prov.close()

    (out / 'dataset.yaml').write_text(
        f'path: {out}\ntrain: images/train\nval: images/val\n'
        'names:\n  0: sock\n')
    print(f'\n== pseudo-label dataset {out} ==')
    print(f'images {stats["imgs"]} (empty/negative {stats["empty"]}) '
          f'boxes {stats["boxes"]} static(claw)-dropped '
          f'{stats["static_dropped"]} ambiguous-skipped '
          f'{stats.get("ambiguous_skipped", 0)} '
          f'dino-failures {stats["dino_fail"]}')
    print('train with: yolo detect train data=dataset.yaml '
          'model=yolov8s.pt  (GPU box)')


if __name__ == '__main__':
    main()
