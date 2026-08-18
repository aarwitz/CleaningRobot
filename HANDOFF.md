# Handoff — sock/clothes-collecting robot + π0 flywheel

Living doc for the next agent. Read `CLAUDE.md` first (architecture + the
HARD RULE), then this. Branch: `sock-demos-and-pi-finetune`, **committed and
pushed** through the 2026-08-16 evening session. Historical session detail
lives in §10; everything above it describes the CURRENT state.

---

## 0. The one rule that matters

**All robot motion goes through `scripts/robot`.** No ad-hoc `docker exec`
python, no hand-publishing to `/teleop/action` or `/cmd_vel`, no calling
`pick_pipeline.py` / `pi_bridge.py` directly. This was an explicit operator
demand after a session of improvised invocations:

> "you are always running almost random pipeline... how can we make sure you
> only have certain modes for running the robot"

Modes (each: flock mutex, dependency preflight, audit line to
`~/robot_runs.log`):

| Mode | Effect |
|---|---|
| `robot status` | health snapshot of profile, teleop link, wrist cam, DINO tunnel, π server. Moves nothing. |
| `robot pick --object X --prompt "Y" [--wrist-detector yolo] [--record] [--episodes N] [--gz Z] [--hover "x,y"] [--drop-at "x,y"]` | the teacher pick. `--wrist-detector yolo` (recommended) = on-device socks2 refine + head-scout-aimed hovers; default `dino` = legacy tunnel refine. `--hover` validated to 170≤r≤345, \|y\|≤100. |
| `robot pi --prompt "..." [--execute] [--watch-s S]` | π0 policy episode; inference-only unless `--execute` |
| `robot calibrate` | re-measure the wrist px→mm Jacobian |
| `robot anchor --object X --prompt "Y" [--gz Z] [--hover "x,y"]` | self-anchor via grasp-drop-observe. Anchors are POSE-DEPENDENT — learn from the hover you will pick from. |
| `robot classic [--go\|--halt\|--off] [--yes]` | classic autonomous profile: fine-tuned YOLO 2D→depth→3D→arm_bridge loop via compose override (teleop OFF — serial conflict); bring-up gates the loop, `--go` arms it. `robot estop` is profile-aware. |
| `robot viz` | repair the operator console's data path. No motion. |
| `robot halt` | safe stop: SIGTERM pipelines (their traps lift first), disarm π bridge (log-verified), lift to tucked pose unless estop latched. |
| `robot stow` | tucked safe pose, gripper open |
| `robot estop` | software E-STOP; skips the mutex so it always fires |

Missing capability → **add a small vetted mode**, never bypass. Read-only
observation (`ros2 topic echo`, grabbing frames) is fine outside the wrapper.

---

## 1. Where we actually are (2026-08-16 evening)

**Teacher (works, and is now mostly on-device):** `robot pick
--wrist-detector yolo` runs the fused pipeline:

```
head_scout   head socks2 2D + floor-plane ray-cast -> arm (x,y) targets,
             INCLUDING inside the head cam's <300mm depth-blind band;
             aims the wrist hover, rides into refine as a HARD LOCK
refine       wrist socks2 + anchor/Jacobian servoing, guarded by:
             counter-prompt arbitration (DINO; fails open),
             claw filter + parallax probe, jump gate (>150mm corrections),
             off-target gate (>200mm from the scouted target),
             elongated-object near-end retarget, stamp-gated freshness
grasp        sweep at gz (precedence: --gz > taught bank > scout -203)
verify       HIGH pose first, then claw-region pixel-diff vs an empty-claw
             session-start reference (empty ~2-7, held >14, overwhelming
             >40) AND head floor-clear. Detector-free holds-check.
```

Certified 2026-08-16: staged occluded-sock test passed; 5-episode
unattended batch ran 2 held / 3 missed with **all labels frame-audited
truthful**; final aimed-hover pick held with claw-diff 47.

**Student (does not work yet):** the fine-tuned π0 checkpoint. Plumbing
fully wired and serving; the policy reaches but does not close the grip
(head-cam-only fine-tune cannot see the grasp — see §6 and next step §7.1).

**Dataset:** 116 episodes in `demos/picks/` (47 success / 69 miss; 78 sock,
31 rabbit, 7 fish) + 15 older `demos/ep_*`. Both cameras recorded per
episode. Labels are hand-audited; corrections live in `note:` fields —
including five detector-era false successes corrected on 2026-08-16.
**Misses are kept on purpose** — recovery data is what the policy lacks.

**Pseudo-label set:** `/demos/pseudo_v1` (built 2026-08-16): 1117 images
(417 pure negatives), 1158 boxes, 103 static claw boxes dropped, 161
ambiguous frames skipped, YOLO layout + `dataset.yaml` + provenance.jsonl.
This is the socks3 training input (§7.3).

**Hardware state:** base motors intentionally UNPOWERED (arm on wall PSU).
Arm-only; objects must be inside the reach annulus (~170–320mm).
`--allow-drive` gates any base motion and is off by default.

---

## 2. Target architecture and where each piece stands

```
runtime detector      : fine-tuned YOLO on-device      DONE for the teacher
                        (socks2 on BOTH cameras via yolo_trt_py; the Isaac
                        ROS head path additionally exists for the classic
                        profile)
open-vocab fallback   : GroundingDINO on RSL           in use as ARBITER
                        (counter-prompt), head coarse for non-wrist-only,
                        and the pseudo-labeler
pseudo-labeler        : DINO -> YOLO datasets          DONE (pseudolabel.py)
manipulation          : π0 fine-tune owns the grasp    NOT YET (retrain
                        queued, now dual-camera)
navigation            : engineered encoder/Nav2 stack  base unpowered
```

Operator, verbatim (the design intent):

> "the full pipeline i was running for cleaningrobot before with the
> yolo3ddetector actually used the finetuned yolo for which i achieved a
> higher accuracy, i was not planning on making calls to rsl groundingdino —
> i only used that for pseudolabeling."

The long-term product loop: wander → detect → approach → **π0 grasp** →
carry → place in basket, with the flywheel continuously distilling DINO
pseudo-labels into better on-device models.

---

## 3. Hard-won constants and facts — do not "clean these up"

- **Wrist anchor** `/demos/wrist_anchor.json`: currently `(346,385)` n=4,
  self-learned 2026-08-16 (the earlier `(243,251)` was wrong for the current
  camera mount and produced six straight centered-refine empty grasps).
  Anchors are POSE-DEPENDENT; self-anchor from the hover you will pick from.
  Corrections must be **rotated by base yaw** (`R(atan2(by,bx))`) — J was
  measured at yaw≈0.
- **Reach**: `FLOOR_REACH_R = 320.0` in `pick_pipeline.py` (operator-reduced
  from the empirical ~340; `send()` silently CLAMPS out-of-range commands, so
  an unguarded far target short-picks every time). Reach gate + near-end
  retarget + in-reach-neighbor + "needs a base move" skip all key off it.
- **Grasp depth `gz`** ladder: scrunched −195, flattened −203, floor −212.
  Precedence in code: `--gz` > per-object taught bank > scout default −203.
  Depth-loft per target is UNMEASURABLE in the pick zone (RealSense
  min-range ~300mm — measured 2026-08-16), hence a fixed default.
  Pick-drop cycles UNROLL socks; hold rate decays mid-session — look at the
  scene before blaming code.
- **Verify** (rebuilt 2026-08-16 after five detector false successes): the
  holds-check is DETECTOR-FREE — command the standard high pose
  (255,0,60), then mean |gray diff| of the claw region [260:460,180:460]
  vs an empty-claw reference captured at session start. Empty ~2–7,
  held >14, overwhelming >40. Rationale: the scene permanently contains a
  dark sock-shaped claw, so every detector referee eventually lies (DINO
  mega-boxes; socks2 scores the claw at low conf). Head floor-clear (DINO,
  bearing-filtered to the pick) must also agree unless the wrist diff is
  overwhelming. Reference and verify are both grip-closed at the same pose.
- **socks2.onnx expects BGR input** (measured: BGR 0.54–0.81 vs RGB
  0.05–0.67 on identical frames) — `yolo_trt_node` runs it with
  `bgr_input:=true`. A fresh ultralytics-trained socks3 will be standard
  RGB: flip that flag or it will be blind.
- **TensorRT + numpy**: a transposed (non-contiguous) blob handed to
  `data_ptr()` is read as dense NCHW — channel-scrambled input, ~0.000
  scores on everything. `_preprocess` now returns `ascontiguousarray`.
- **Detection freshness ≠ message freshness**: yolo dets are gated on the
  IMAGE HEADER STAMP (frame captured after the request), not arrival time —
  a mid-lift frame once scored as a hold.
- Depth is blind below ~300mm — why wrist-only mode and the floor-plane
  scout exist at all. The floor plane is fit from the VALID band, full
  width, robust refit, per-round retry (transient depth garbage after arm
  moves spikes resid).
- Scout projection carries a bias vs actual picks (~(+65,+68)mm observed);
  refine's off-target gate is 200mm until the bias is fit from the
  `scout_target` field now recorded in episode meta, then re-tighten to 100.

---

## 4. π0 status — the honest version

- **Checkpoint:** `~/checkpoints/pi0_sock_v1/4999` on the Jetson (8.8 GB),
  rsynced to RSL. Serving on RSL:
  ```
  cd ~/openpi && uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_roarm_sock_cartesian_lora \
    --policy.dir=$HOME/checkpoints/pi0_sock_v1/4999
  ```
  Config injection (openpi `15a9616`): copy `roarm_policy.py` into
  `src/openpi/policies/`, append `roarm_inject_block.py` to
  `src/openpi/training/config.py`. Health check: `curl :8000` → **426**.
  ~348 ms/inference, (50,4) cartesian chunks.
- **Behavior:** reaches, hovers touching the sock, does not close. 0 grasps
  in the current room (July result was the training room). Head-cam-only
  fine-tune + 11-minute dataset with no recovery data.
- **The dual-camera retrain path is fully wired** (2026-08-15/16):
  `convert_roarm_to_lerobot.py --wrist --include-misses` (the converter used
  to silently skip misses), NEW config `pi0_roarm_sock_cartesian_wrist_lora`
  (repo `roarm_sock_cartesian_wrist`; new name so the served v1 keeps its
  exact inputs), `pi_bridge` `wrist_topic` param for serving (refuses to
  infer on a missing wrist frame). All picks episodes have paired wrist
  frames except ep_0035.
- "LeRobot" is the *dataset format* openpi trains from, not a robot.

---

## 5. Bugs found the hard way, and what they teach

1. **`robot estop` was a NO-OP** (published to the wrong topic; the handler
   silently dropped it). *Verify safety paths actually fire.*
2. **rosbridge died silently** (no respawn; console showed "no signal"
   forever). Fixed in launch + `robot viz` live repair.
3. **Killing a batch script is not a safe stop** — servos hunt against the
   floor at grasp depth. → `robot halt` + SIGINT/SIGTERM traps in
   `pick_pipeline`.
4. **`docker exec` timeouts kill the client, not the process** — bound
   everything with in-container `timeout`.
5. **Self-written evaluators lie** unless structurally prevented: a π0
   "5/5" was fabricated by a holds-check from an arbitrary pose; five more
   false successes came from detector-based verify in one afternoon. The
   pick verify is fixed (high pose + empty-claw reference); **the π0 eval
   harness itself still needs the same treatment** (§7.2).
6. **Label integrity is enforceable**: frame-audit every "held" until the
   verify path has earned trust; corrections belong in meta `note:` fields.

---

## 6. Next steps, in priority order

1. **Retrain π0 dual-camera** on the GPU box:
   `./gpu_bootstrap.sh --wrist --include-misses --demos demos/picks --train`
   (~8 h / ~$4 RunPod; needs neither robot nor operator). 116 episodes now
   vs the 50 the v1 saw, wrist view included, recovery data included.
2. **Rebuild the π0 evaluator** on the new verify doctrine (high pose +
   claw-diff reference + void trials with failed staging), then re-baseline.
3. **Train socks3** from `/demos/pseudo_v1` (ultralytics, GPU box; see
   `scripts/pseudolabel.py` header). Fixes the wrist-view OOD failures
   (claw-as-sock, sock+claw grouped boxes). Remember: RGB, flip
   `bgr_input:=false`.
4. **Fit the scout bias** once enough `scout_target`-vs-pick pairs exist in
   meta; re-tighten the off-target gate 200→100.
5. **Restart the container** at a convenient moment: `ENABLE_WRIST_YOLO=true`
   is now in compose/entrypoint/launch, which makes the yolo nodes+republish
   launch-managed (they currently ALSO run ad-hoc from the bring-up session —
   after restart the ad-hoc copies are gone and the launch owns them).
6. When the base is powered: fish/flip-flop area + wicker basket → mobile
   pick→carry→place (tasks #24/#25).

---

## 6b. In flight (2026-08-18)

- **π0 v2 training RUNNING on RunPod**: pod `5t8jza01ekta74` (A40, $0.44/hr,
  ssh via jetson key `~/.ssh/id_ed25519_runpod` -> root@69.30.85.129:22057).
  `pi0_roarm_sock_cartesian_wrist_lora --exp-name sock_v2_wrist`, 5000 steps,
  wandb disabled, log `/workspace/train.log`, data checksum-verified
  (116 eps). Bootstrap's norm-stats integration test PASSED. When done:
  checkpoint at `~/openpi/checkpoints/pi0_roarm_sock_cartesian_wrist_lora/
  sock_v2_wrist/<step>` -- rsync to RSL, serve with the SAME config name and
  `pi_bridge` `wrist_topic:=/wrist_cam/image_raw/compressed`.
- **socks3 queued on the same pod** after pi0 finishes: `/workspace/
  pseudo_v1.tar` is up; pip install ultralytics, `yolo detect train
  data=pseudo_v1/dataset.yaml model=yolov8s.pt epochs=60 imgsz=640`,
  export ONNX. Remember: RGB model -> `bgr_input:=false`.
- **`robot eval` mode + `scripts/pi_eval.py`** (the honest evaluator, §5.5
  doctrine): scout-staged trials (voids never count), log-verified policy
  stop, claw-diff verify from the high pose. BLOCKED on arm power for the
  v1 baseline -- run `robot eval --trials 5` once the PSU is on.
- Container restarted: launch-managed yolo nodes verified live (4.4 Hz).
- STOP THE POD when done: `curl -X POST -H "Authorization: Bearer $(cat
  ~/.runpod_key)" https://rest.runpod.io/v1/pods/5t8jza01ekta74/stop`
  (then DELETE to stop storage billing).

## 6c. Findings 2026-08-18 evening

- **socks2 is light-blind**: same socks, same spots — daylight scores
  0.79–0.86, evening scores 0.01–0.19 (head cam). The scout therefore finds
  no targets at night and `robot eval` correctly VOIDS all trials (5/5 void,
  nothing fabricated — the harness works). Consequences:
  (a) v1 baseline eval must run in DAYLIGHT for a fair before/after vs v2;
  (b) socks3 must train with strong HSV/brightness augmentation (ultralytics
  defaults help; consider augmenting pseudo_v1 with gamma-jittered copies);
  (c) longer-term: pseudo-label episodes collected across lighting.
- Container recreation wipes ad-hoc pip deps: pi_bridge needs
  `pip3 install msgpack websockets` after every recreate (put in Dockerfile
  eventually). pi_bridge is still ad-hoc (not launch-managed) by design —
  it depends on the RSL tunnel.
- Training progress: openpi with wandb disabled logs NO loss lines — only
  tqdm progress. Judge checkpoints by the eval harness, not the curve.

## 7. Operating notes / gotchas

- Config truth lives in `docker/docker-compose.yml` env vars, **not** the
  launch file's `DeclareLaunchArgument` defaults (those are stale).
- DINO tunnel (still needed for counter-prompt, head coarse, pseudo-label):
  `ssh -f -N -L 8002:localhost:8002 -L 8000:localhost:8000 -o HostKeyAlias=RSL aaron@100.110.113.91`
  Detector paths fail OPEN without it.
- The yolo nodes: wrist `/wrist_yolo/detections` (conf 0.22, bgr), head
  `/head_yolo/detections` (conf 0.30, bgr), plus an `image_transport
  republish` for `/wrist_cam/image_raw` (the wrist cam only publishes
  compressed). Engine `/models/socks2_py.plan` is the python node's own
  FP32 build — do not point it at the Isaac `socks2.plan`.
- Episodes under `demos/` are root-owned (written from the container) —
  edit/delete via the container.
- Optical flow (Farneback) is useless on the textureless wood floor.
- The RoArm firmware echoes IK pose even with servos unpowered — encoder
  feedback does NOT prove motion. Only vision or the operator confirms.
- `pgrep -f` inside `docker exec bash -c` matches its own command line —
  use the `[b]racket` trick.
- `nohup` python logs are block-buffered — an empty log does not mean a
  dead process; check provenance/output files.

## 8. Standing operator constraints

- Never print the contents of `~/.runpod_key` or `~/.dino_key`.
- Do not store anything about Railway (operator asked for it to be
  forgotten).
- The leaked `id_ed25519` is treated as compromised.
- Base motion requires an explicit operator grant.
- **Dataset labels must stay truthful** — a miss recorded as a success
  poisons the fine-tune. This is the one class of mistake that is never
  acceptable; false-positive grasps and wasted cycles are fine and even
  useful.

---

## 9. The teacher's guard rationale (kept because each cost a session)

- **Parallax probe**: claw fingers ride the camera (static px across a
  jog); real floor objects shift. Dark-on-dark socks beside the claw are
  invisible without it.
- **Counter-prompt arbitration**: confidence alone cannot separate junk
  from socks (creamer 0.42 > white sock 0.38); and single-class YOLO is not
  immune — a plastic bag FULL of socks detects as sock through the plastic.
  Min-area 6k px² + "metal cup. mug. jar. bottle. plastic bag." IoU claim.
- **Jump gate** (>150mm implied correction, only once a lock exists): a
  frame-edge det steered the arm off a locked target; first-pass far dets
  are what scanning is FOR, so unlocked passes are exempt.
- **Off-target gate** (>200mm from the scout HARD lock only): cross-camera
  consistency; parallax self-locks are sorting preferences, not gates.
- **Near-end retarget**: a radially-lying sock's centroid maps ~10cm past
  its graspable near end; if the centroid exceeds reach but the box near
  end (high-v) is inside, grasp the near end.
- **Wrist-stale guard**: refuse corrections when the wrist view did not
  change across a hover move.

## 10. Session log (condensed provenance)

- **2026-08-15**: hardened `scripts/robot` wrapper; parallax probe;
  distractor filter; reach gate; `--wrist-only/--self-anchor/--calibrate-
  wrist/--gz/--drop-at`; wrist_cam reader thread + freeze watchdog;
  flywheel_relay; arm_link ttyUSB glob; rosbridge respawn.
- **2026-08-16 morning**: `robot classic` (compose-override profile),
  `robot halt` (+ pipeline signal traps), dual-camera π0 training path
  (converter/config/policy/bridge), first commits pushed.
- **2026-08-16 afternoon**: socks2 resurrected on-device (contiguity + BGR);
  wrist+head yolo_trt instances; stamp-gated freshness; anchor re-learned
  (346,385); jump/off-target gates; near-end retarget; FLOOR_REACH_R 320;
  head_scout floor-plane fusion (validated offline via
  `scripts/scout_validate.py`, which feeds the SUDS overlays); verify
  rebuilt detector-free after five audited false successes (ep_0098/0099/
  0107/0108/0109 corrected); reject-annotated overlays (operator request).
- **2026-08-16 evening**: staged occluded-sock reliability test PASSED;
  5-episode batch 2/5 held, labels certified; hard_lock separation; scout
  bias identified (~(+65,+68)) and gate widened with meta logging;
  `pseudolabel.py` + `/demos/pseudo_v1` built (1117 imgs / 1158 boxes /
  417 negatives); yolo nodes moved into the launch stack
  (`ENABLE_WRIST_YOLO`); scout gz default made bank-respecting.
