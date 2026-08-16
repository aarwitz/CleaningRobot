#!/usr/bin/env python3
"""Build a proper LeRobot dataset from raw RoArm demos, for openpi fine-tuning.

RUN THIS ON THE GPU BOX (where `lerobot` + `openpi` are installed), not on the
Jetson. `scripts/demos_to_lerobot.py` on the robot writes a JSONL structure that
is fine for INSPECTION, but openpi loads data through
`lerobot.common.datasets.LeRobotDataset`, which needs the real v2 layout
(parquet shards + meta/stats.json + meta/tasks.jsonl computed by lerobot). Only
lerobot itself writes that correctly, hence this second converter.

It reads the raw episodes (demos/ep_XXXX/{meta.json, traj.jsonl, frames/*.jpg})
straight from the robot -- rsync the `demos/` folder over -- so there is a
single source of truth and no intermediate to keep in sync.

State/action space (choose to match the openpi config you train):
  joint      [base, shoulder, elbow, gripper]  (arm feedback, rad)
  cartesian  [x, y, z, gripper]                 (maps 1:1 to a T:1041 command)
Action = the next step's state (position control), i.e. the reachable target the
scripted trajectory actually achieved.

  python convert_roarm_to_lerobot.py --demos ./demos --repo-id roarm_sock \
      --space cartesian --prompt "pick up the sock"

Output goes to the local LeRobot cache (~/.cache/huggingface/lerobot/<repo-id>),
which is exactly what the openpi TrainConfig's repo_id then points at.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

# lerobot is a GPU-box dependency; import lazily so --help works anywhere.
try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
except Exception as e:  # pragma: no cover
    LeRobotDataset = None
    _IMPORT_ERR = e

SPACES = {
    'joint': ['base', 'shoulder', 'elbow', 'gripper'],
    'cartesian': ['x', 'y', 'z', 'gripper'],
}


def row_state(r, space):
    p = r.get('pose') or {}
    if space == 'cartesian':
        return [p.get('x'), p.get('y'), p.get('z'), p.get('t')]
    j = r.get('joints')
    if isinstance(j, dict):
        return [j.get('b'), j.get('s'), j.get('e'), p.get('t')]
    if isinstance(j, (list, tuple)):
        return list(j)[:4]
    return [None, None, None, None]


def load_episode(ep_dir, space, wrist=False):
    meta = json.loads((ep_dir / 'meta.json').read_text())
    rows = [json.loads(l) for l in (ep_dir / 'traj.jsonl').read_text().splitlines() if l]
    rows = [r for r in rows
            if r.get('frame') and all(v is not None for v in row_state(r, space))]
    if wrist:
        # wrist frame = same filename under frames_wrist/ (DualRecorder pairs
        # them by index). Drop rows whose wrist frame is missing; the caller
        # skips the episode if too few survive (e.g. ep_0035: wrist cam died,
        # dir exists but is empty).
        rows = [r for r in rows
                if (ep_dir / 'frames_wrist' / r['frame']).exists()]
    return meta, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demos', nargs='+', default=['./demos'],
                    help='one or more episode roots (e.g. demos demos/picks)')
    ap.add_argument('--repo-id', default='roarm_sock')
    ap.add_argument('--space', choices=list(SPACES), default='cartesian')
    ap.add_argument('--prompt', default='pick up the sock')
    ap.add_argument('--fps', type=int, default=None,
                    help='default: measured from the recorded timestamps')
    ap.add_argument('--wrist', action='store_true',
                    help='add observation.images.wrist from frames_wrist/. '
                         'Episodes without paired wrist frames are SKIPPED. '
                         'The grasp happens inside the head cam\'s depth-blind '
                         'zone (<300mm), so head-only training asks the policy '
                         'to close a grip it cannot see.')
    ap.add_argument('--include-misses', action='store_true',
                    help='keep success=false episodes (recovery data). The '
                         'action targets are still the real achieved states; '
                         'labels stay truthful either way.')
    args = ap.parse_args()

    if LeRobotDataset is None:
        raise SystemExit(
            f'lerobot not importable ({_IMPORT_ERR}). Run this on the GPU box '
            'with openpi/lerobot installed, not on the robot.')

    eps = []
    for root in args.demos:
        eps += sorted(d for d in Path(root).glob('ep_*')
                      if (d / 'traj.jsonl').exists())
    if not eps:
        raise SystemExit(f'no episodes under {args.demos}')

    fps = args.fps
    if fps is None:
        dts = []
        for ep in eps:
            _, rows = load_episode(ep, args.space)
            dts += [b['ts'] - a['ts'] for a, b in zip(rows, rows[1:])]
        fps = int(round(1.0 / float(np.median(dts)))) if dts else 15
        print(f'measured sampling rate ~= {fps} Hz')

    names = SPACES[args.space]
    features = {
        'observation.images.exterior': {
            'dtype': 'image', 'shape': (480, 640, 3),
            'names': ['height', 'width', 'channel']},
        'observation.state': {
            'dtype': 'float32', 'shape': (4,), 'names': names},
        'action': {
            'dtype': 'float32', 'shape': (4,), 'names': names},
    }
    if args.wrist:
        features['observation.images.wrist'] = {
            'dtype': 'image', 'shape': (480, 640, 3),
            'names': ['height', 'width', 'channel']}
    ds = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=fps,
        robot_type=f'roarm_m2s_{args.space}',
        features=features,
        image_writer_threads=8,
        image_writer_processes=2,
    )

    kept = n_success = n_miss = 0
    for ep in eps:
        meta, rows = load_episode(ep, args.space, wrist=args.wrist)
        success = bool(meta.get('success'))
        if not success and not args.include_misses:
            print(f'skip {ep.name} (miss; use --include-misses to keep)')
            continue
        if len(rows) < 4:
            print(f'skip {ep.name} '
                  f'({"no paired wrist frames" if args.wrist else "too short"})')
            continue
        states = np.array([row_state(r, args.space) for r in rows], dtype=np.float32)
        actions = np.vstack([states[1:], states[-1:]])       # next-state targets
        prompt = meta.get('prompt', args.prompt)
        for i, r in enumerate(rows):
            frame = {
                'observation.images.exterior': np.asarray(
                    Image.open(ep / 'frames' / r['frame']).convert('RGB')),
                'observation.state': states[i],
                'action': actions[i],
                'task': prompt,
            }
            if args.wrist:
                frame['observation.images.wrist'] = np.asarray(
                    Image.open(ep / 'frames_wrist' / r['frame']).convert('RGB'))
            ds.add_frame(frame)
        ds.save_episode()
        kept += 1
        n_success += success
        n_miss += not success
        print(f'{ep.name} -> {len(rows)} steps'
              + ('' if success else ' (miss)'))

    print(f'\n{kept} episodes ({n_success} success / {n_miss} miss) -> '
          f'LeRobot dataset "{args.repo_id}" '
          f'({args.space} space, {fps} Hz'
          f'{", wrist included" if args.wrist else ""})')
    print('norm stats are computed by openpi: '
          f'uv run scripts/compute_norm_stats.py <your-config>')


if __name__ == '__main__':
    main()
