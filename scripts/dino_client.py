#!/usr/bin/env python3
"""Thin client for the GroundingDINO service on RSL (Dockerized_GroundingSAM).

The service is the autonomous half of the operator's pseudolabeling pipeline:
open-vocabulary detection with the same constraint set the human reviewer was
approximating (ROI, area band, IoU, per-class thresholds). In our controlled
collection scene those constraints replace the reviewer entirely.

The API listens on RSL at 127.0.0.1:8002 (host-published; NOT reachable from
the LAN directly — the thing on :8000 is unrelated). We reach it through an
SSH tunnel, which systemd or the caller keeps alive:

  ssh -N -L 8002:localhost:8002 aaron@RSL &

Auth: the DEMO_API_KEY from the container env, cached at ~/.dino_key (0600) by
fetch_key(). Never printed.

  python3 dino_client.py /tmp/frame.png "white sock,grey sock,sock"
"""
import json
import os
import subprocess
import sys
import time
import urllib.request
import uuid

DEFAULT_URL = os.environ.get('DINO_URL', 'http://localhost:8002')
KEY_PATH = os.path.expanduser('~/.dino_key')

# Sock-scene constraint set (tuned 2026-07-24 on live frames): the pick surface
# occupies the lower ~60% of the image; a scrunched sock is ~6-20k px^2; the
# arm column false-positive is ~75k px^2 and crosses the top edge.
SOCK_PROMPT = 'white sock,grey sock,sock,rolled sock'
SOCK_ROI = (32, 168, 608, 480)          # x0,y0,x1,y1 PIXELS in the 640x480
                                        # frame (server scales with the image):
                                        # lower band = the pick surface
SOCK_AREA = (3000, 35000)


def fetch_key():
    """Cache the API key locally (via ssh) so collection doesn't shell out."""
    if os.path.exists(KEY_PATH):
        return open(KEY_PATH).read().strip()
    key = subprocess.run(
        ['ssh', 'aaron@RSL',
         'docker exec dockerized_groundingsam-api-1 printenv DEMO_API_KEY'],
        capture_output=True, text=True, timeout=20).stdout.strip()
    if not key:
        raise RuntimeError('could not fetch DEMO_API_KEY from RSL')
    fd = os.open(KEY_PATH, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.write(fd, key.encode()); os.close(fd)
    return key


def detect(image_path, prompt, confidence=0.15, roi=None, area=None,
           url=DEFAULT_URL, timeout=30):
    """Return detections [{label, confidence, box:[x0,y0,x1,y1]}, ...] sorted
    by confidence, filtered by the ROI/area constraints server-side."""
    key = fetch_key()
    boundary = uuid.uuid4().hex
    img = open(image_path, 'rb').read()

    def part(name, value):
        return (f'--{boundary}\r\nContent-Disposition: form-data; '
                f'name="{name}"\r\n\r\n{value}\r\n').encode()

    body = b''
    body += (f'--{boundary}\r\nContent-Disposition: form-data; name="image"; '
             f'filename="f.png"\r\nContent-Type: image/png\r\n\r\n').encode()
    body += img + b'\r\n'
    body += part('prompt', prompt)
    body += part('confidence', confidence)
    body += part('return_masks', 'false')
    if roi:
        body += part('roi', ','.join(str(v) for v in roi))
    if area:
        body += part('min_area', area[0])
        body += part('max_area', area[1])
    body += f'--{boundary}--\r\n'.encode()

    req = urllib.request.Request(
        f'{url}/api/predict', data=body, method='POST',
        headers={'Content-Type': f'multipart/form-data; boundary={boundary}',
                 'X-API-Key': key})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        resp = json.load(r)
    dets = resp.get('detections', [])
    for d in dets:
        d['box'] = d.get('box') or d.get('bbox')
    dets.sort(key=lambda d: -(d.get('confidence') or d.get('score') or 0))
    return dets


def detect_sock(image_path, **kw):
    """The sock-tuned call: constraint set replaces the human reviewer."""
    return detect(image_path, SOCK_PROMPT, roi=SOCK_ROI, area=SOCK_AREA, **kw)


def box_center(box):
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


if __name__ == '__main__':
    path = sys.argv[1]
    prompt = sys.argv[2] if len(sys.argv) > 2 else SOCK_PROMPT
    t0 = time.time()
    dets = detect(path, prompt, roi=SOCK_ROI, area=SOCK_AREA)
    print(f'{len(dets)} detection(s) in {time.time()-t0:.2f}s')
    for d in dets[:5]:
        print(f"  {d.get('label')} {d.get('confidence', d.get('score')):.2f} "
              f"box={[int(v) for v in d['box']]} center={box_center(d['box'])}")
