#!/usr/bin/env python3
"""Offline smoke test of the fine-tuned pi0 policy server.

Sends ONE real camera frame + a caller-supplied arm state to the server and
sanity-checks the returned action chunk (shape, workspace bounds, gripper
range). Use when the arm is unpowered and the live pi_bridge path (which
correctly refuses to run without real /teleop/state) can't be exercised.

  python3 /scripts/pi_smoke_test.py /tmp/frame.png 255.7 -6.3 -45.1 1.09
"""
import sys

import cv2
import numpy as np

sys.path.insert(0, '/scripts')
from pi_bridge import packb, unpackb, resize_with_pad  # noqa: E402

import websockets.sync.client  # noqa: E402


def main():
    img_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/frame.png'
    state = np.asarray([float(v) for v in sys.argv[2:6]], dtype=np.float32) \
        if len(sys.argv) >= 6 else np.asarray([255.7, -6.3, -45.1, 1.09], np.float32)

    img = cv2.imread(img_path)[:, :, ::-1]
    ws = websockets.sync.client.connect('ws://localhost:8000',
                                        compression=None, max_size=None,
                                        open_timeout=10)
    meta = unpackb(ws.recv())
    print('server metadata:', meta)

    obs = {
        'observation/image': resize_with_pad(img, 224, 224),
        'observation/state': state,
        'prompt': 'pick up the sock',
    }
    import time
    t0 = time.time()
    ws.send(packb(obs))
    resp = ws.recv()
    dt = time.time() - t0
    if isinstance(resp, str):
        print('SERVER ERROR:', resp[:400]); return 1
    actions = np.asarray(unpackb(resp)['actions'])
    print(f'\ninference: {dt*1000:.0f} ms   chunk: {actions.shape}')
    print(f'input state [x,y,z,grip]: {state.tolist()}')
    for name, i in (('first', 0), ('mid', len(actions)//2), ('last', -1)):
        print(f'  {name:5s} action: {[round(float(v),1) for v in actions[i]]}')

    ok = True
    if actions.ndim != 2 or actions.shape[1] != 4:
        print('FAIL: expected (horizon, 4)'); ok = False
    r = np.hypot(actions[:, 0], actions[:, 1])
    print(f'\nranges over chunk: r {r.min():.0f}..{r.max():.0f}  '
          f'z {actions[:,2].min():.0f}..{actions[:,2].max():.0f}  '
          f'grip {actions[:,3].min():.2f}..{actions[:,3].max():.2f}')
    if not (100 < r.min() and r.max() < 400):
        print('WARN: radius outside plausible workspace')
    if not (-250 < actions[:, 2].min() and actions[:, 2].max() < 350):
        print('WARN: z outside plausible workspace')
    if not (0.8 <= actions[:, 3].min() and actions[:, 3].max() <= 3.3):
        print('WARN: gripper outside [1.08, 3.14]')
    print('\nSMOKE TEST', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
