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


def load_episode(ep_dir, space):
    meta = json.loads((ep_dir / 'meta.json').read_text())
    rows = [json.loads(l) for l in (ep_dir / 'traj.jsonl').read_text().splitlines() if l]
    rows = [r for r in rows
            if r.get('frame') and all(v is not None for v in row_state(r, space))]
    return meta, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demos', default='./demos')
    ap.add_argument('--repo-id', default='roarm_sock')
    ap.add_argument('--space', choices=list(SPACES), default='cartesian')
    ap.add_argument('--prompt', default='pick up the sock')
    ap.add_argument('--fps', type=int, default=None,
                    help='default: measured from the recorded timestamps')
    args = ap.parse_args()

    if LeRobotDataset is None:
        raise SystemExit(
            f'lerobot not importable ({_IMPORT_ERR}). Run this on the GPU box '
            'with openpi/lerobot installed, not on the robot.')

    demos = Path(args.demos)
    eps = sorted(d for d in demos.glob('ep_*') if (d / 'traj.jsonl').exists())
    if not eps:
        raise SystemExit(f'no episodes under {demos}')

    fps = args.fps
    if fps is None:
        dts = []
        for ep in eps:
            _, rows = load_episode(ep, args.space)
            dts += [b['ts'] - a['ts'] for a, b in zip(rows, rows[1:])]
        fps = int(round(1.0 / float(np.median(dts)))) if dts else 15
        print(f'measured sampling rate ~= {fps} Hz')

    names = SPACES[args.space]
    ds = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=fps,
        robot_type=f'roarm_m2s_{args.space}',
        features={
            'observation.images.exterior': {
                'dtype': 'image', 'shape': (480, 640, 3),
                'names': ['height', 'width', 'channel']},
            'observation.state': {
                'dtype': 'float32', 'shape': (4,), 'names': names},
            'action': {
                'dtype': 'float32', 'shape': (4,), 'names': names},
        },
        image_writer_threads=8,
        image_writer_processes=2,
    )

    kept = 0
    for ep in eps:
        meta, rows = load_episode(ep, args.space)
        if not meta.get('success') or len(rows) < 4:
            print(f'skip {ep.name}')
            continue
        states = np.array([row_state(r, args.space) for r in rows], dtype=np.float32)
        actions = np.vstack([states[1:], states[-1:]])       # next-state targets
        prompt = meta.get('prompt', args.prompt)
        for i, r in enumerate(rows):
            img = np.asarray(Image.open(ep / 'frames' / r['frame']).convert('RGB'))
            ds.add_frame({
                'observation.images.exterior': img,
                'observation.state': states[i],
                'action': actions[i],
                'task': prompt,
            })
        ds.save_episode()
        kept += 1
        print(f'{ep.name} -> {len(rows)} steps')

    print(f'\n{kept} episodes -> LeRobot dataset "{args.repo_id}" '
          f'({args.space} space, {fps} Hz)')
    print('norm stats are computed by openpi: '
          f'uv run scripts/compute_norm_stats.py <your-config>')


if __name__ == '__main__':
    main()
