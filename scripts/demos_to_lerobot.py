#!/usr/bin/env python3
"""Convert collected demo episodes into a LeRobot-format dataset for openpi
fine-tuning.

Input   demos/ep_XXXX/{meta.json, traj.jsonl, frames/*.jpg}
Output  a LeRobot v2 dataset directory (parquet trajectories + frame videos or
        image folders) that `openpi` training configs can consume.

State/action space for this arm (4 DoF): [base, shoulder, elbow, gripper] in
radians, read back from the arm's own feedback. Actions are the NEXT sampled
state (position control), which is what the scripted trajectories actually
achieved, so the policy learns reachable targets rather than raw commands.

Usage:
  python3 scripts/demos_to_lerobot.py --demos demos --out lerobot_sock \
      --prompt "pick up the sock"

Then on the GPU box (openpi):
  uv run scripts/compute_norm_stats.py <config>
  uv run scripts/train.py <config> --exp-name sock_lora
"""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np


# State/action space definitions. 'joint' matches the arm's own feedback;
# 'cartesian' matches how we actually COMMAND the arm (T:1041 x,y,z,gripper),
# so a policy trained in cartesian space emits values that map straight back to
# an arm command with no forward kinematics -- better for closing the loop.
SPACES = {
    'joint': ['base', 'shoulder', 'elbow', 'gripper'],
    'cartesian': ['x', 'y', 'z', 'gripper'],
}


def row_state(r, space='joint'):
    """4-DoF proprioceptive state as a flat vector, per `space`.

    sock_cycle records joints as a dict {b, s, e} (arm joints, rad) and the
    gripper angle separately in pose.t (1.08 open .. 3.14 closed); pose also
    holds cartesian x,y,z (mm). Earlier collectors stored joints as a flat
    list, so the joint reader handles both.
    """
    p = r.get('pose') or {}
    if space == 'cartesian':
        return [p.get('x'), p.get('y'), p.get('z'), p.get('t')]
    j = r.get('joints')
    if isinstance(j, dict):
        return [j.get('b'), j.get('s'), j.get('e'), p.get('t')]
    if isinstance(j, (list, tuple)):
        return list(j)[:4]
    return [None, None, None, None]


def load_episode(ep_dir, space='joint'):
    meta = json.loads((ep_dir / 'meta.json').read_text())
    rows = [json.loads(l) for l in (ep_dir / 'traj.jsonl').read_text().splitlines() if l]
    rows = [r for r in rows
            if r.get('frame') and all(v is not None for v in row_state(r, space))]
    return meta, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demos', default='demos')
    ap.add_argument('--out', default='lerobot_sock')
    ap.add_argument('--prompt', default=None,
                    help='override the per-episode prompt')
    ap.add_argument('--fps', type=float, default=None,
                    help='default: measured from the recorded timestamps')
    ap.add_argument('--space', choices=list(SPACES), default='joint',
                    help="state/action space: 'joint' [b,s,e,grip] (arm "
                         "feedback) or 'cartesian' [x,y,z,grip] (maps straight "
                         "to a T:1041 command)")
    ap.add_argument('--successful-only', action='store_true', default=True)
    args = ap.parse_args()

    demos = Path(args.demos)
    out = Path(args.out)
    if out.exists():
        shutil.rmtree(out)
    (out / 'data' / 'chunk-000').mkdir(parents=True)
    (out / 'images').mkdir(parents=True)

    eps = sorted(d for d in demos.glob('ep_*') if (d / 'traj.jsonl').exists())
    kept, total_frames = 0, 0
    episode_index = []

    fps = args.fps
    if fps is None:                 # serial feedback paces sampling, not the loop
        dts = []
        for ep in eps:
            _, rows = load_episode(ep, args.space)
            # row timestamp is 'ts' (pose.t is the gripper angle -- name clash)
            dts += [b['ts'] - a['ts'] for a, b in zip(rows, rows[1:])]
        fps = round(1.0 / float(np.median(dts)), 2) if dts else 10.0
        print(f'measured sampling rate: {fps} Hz')

    for ep in eps:
        meta, rows = load_episode(ep, args.space)
        if args.successful_only and not meta.get('success'):
            print(f'skip {ep.name}: failed ({meta.get("failure", "?")})')
            continue
        if len(rows) < 4:
            print(f'skip {ep.name}: too few usable rows')
            continue

        states = np.array([row_state(r, args.space) for r in rows], dtype=np.float32)
        # position-control actions: the state actually reached next step
        actions = np.vstack([states[1:], states[-1:]])
        prompt = args.prompt or meta.get('prompt', 'pick up the sock')

        ep_img_dir = out / 'images' / f'episode_{kept:06d}'
        ep_img_dir.mkdir(parents=True)
        frames = []
        for i, r in enumerate(rows):
            src = ep / 'frames' / r['frame']
            dst = ep_img_dir / f'{i:06d}.jpg'
            shutil.copyfile(src, dst)
            frames.append(str(dst.relative_to(out)))
            total_frames += 1

        recs = [{
            'episode_index': kept,
            'frame_index': i,
            'index': total_frames - len(rows) + i,
            'timestamp': round(i / fps, 4),
            'observation.state': states[i].tolist(),
            'action': actions[i].tolist(),
            'observation.images.exterior': frames[i],
            'task': prompt,
            'phase': rows[i]['phase'],
        } for i in range(len(rows))]

        with (out / 'data' / 'chunk-000' / f'episode_{kept:06d}.jsonl').open('w') as f:
            for r in recs:
                f.write(json.dumps(r) + '\n')
        episode_index.append({'episode_index': kept, 'length': len(rows),
                              'tasks': [prompt], 'source': ep.name})
        kept += 1
        print(f'{ep.name} -> episode_{kept - 1:06d}: {len(rows)} steps')

    (out / 'meta').mkdir(exist_ok=True)
    (out / 'meta' / 'episodes.jsonl').write_text(
        '\n'.join(json.dumps(e) for e in episode_index))
    names = SPACES[args.space]
    (out / 'meta' / 'info.json').write_text(json.dumps({
        'codebase_version': 'v2.0',
        'robot_type': f'roarm_m2s_{args.space}',
        'space': args.space,
        'fps': fps,
        'total_episodes': kept,
        'total_frames': total_frames,
        'features': {
            'observation.state': {'dtype': 'float32', 'shape': [4], 'names': names},
            'action': {'dtype': 'float32', 'shape': [4], 'names': names},
            'observation.images.exterior': {'dtype': 'image', 'shape': [480, 640, 3]},
        },
    }, indent=1))
    print(f'\n{kept} episodes, {total_frames} frames -> {out}')
    if kept == 0:
        print('nothing converted — collect successful episodes first')


if __name__ == '__main__':
    main()
