# Handoff — sock/clothes-collecting robot + π0 flywheel

Written 2026-08-15 for the next agent picking this up. Read `CLAUDE.md` first
(architecture + the HARD RULE), then this. Branch: `sock-demos-and-pi-finetune`.
**Everything described below is uncommitted work in the working tree.**

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
| `robot status` | health snapshot of teleop link, wrist cam, DINO tunnel, π server. Moves nothing. |
| `robot pick --object X --prompt "Y" [--record] [--drop-at "x,y"] [--gz Z] [--episodes N]` | scripted DINO wrist-only pick |
| `robot pi --prompt "..." [--execute] [--watch-s S]` | π0 policy episode; inference-only unless `--execute` |
| `robot calibrate` | re-measure the wrist px→mm Jacobian |
| `robot anchor --object X --prompt "Y"` | self-anchor via grasp-drop-observe |
| `robot classic [--go\|--halt\|--off] [--yes]` | classic autonomous profile: fine-tuned YOLO 2D→depth→3D→arm_bridge loop. Compose-override profile swap (teleop OFF — serial conflict); bring-up gates the loop via `/arm_bridge/set_active`, `--go` arms it. `robot estop` is profile-aware: in classic it gates the loop (teleop estop path doesn’t exist). |
| `robot viz` | repair the operator console's data path. No motion. |
| `robot halt` | safe stop: SIGTERM pipelines (their traps lift first), disarm π bridge (log-verified), lift to tucked pose (grip closed) unless estop latched. Built + live-verified 2026-08-16. |
| `robot stow` | tucked safe pose |
| `robot estop` | software E-STOP; skips the mutex so it always fires |

Missing capability → **add a small vetted mode**, never bypass. Read-only
observation (`ros2 topic echo`, grabbing frames) is fine outside the wrapper.

---

## 1. Where we actually are

**Two pipelines exist. One works, one is the goal.**

- **Teacher (works):** `scripts/pick_pipeline.py --wrist-only` — GroundingDINO
  on the RSL box (SSH tunnel :8002) + wrist-camera visual servoing. This is
  what reliably picks socks today and what generates training data.
- **Student (does not work yet):** the fine-tuned π0 checkpoint. Plumbing is
  fully wired and serving; the policy itself does not complete grasps in the
  current room.

**Dataset:** 78 episodes in `demos/picks/` (36 success / 42 miss; 40 sock,
31 rabbit, 7 fish) + 15 older `demos/ep_*` from the `sock_cycle` collector that
fed the first fine-tune. Labels have been hand-audited from frames more than
once; several carry `note:` fields recording operator corrections. **Misses are
kept on purpose** — recovery data is exactly what the policy lacks.

**Hardware state during all recent work:** base motors intentionally UNPOWERED
(arm on wall PSU). Everything is arm-only; the operator places objects within
reach. `--allow-drive` gates any base motion and is off by default.

---

## 2. The ideal pipeline (target architecture)

The operator's intended design, which we drifted from and should return to:

```
runtime detector      : fine-tuned YOLO on-device  (models/socks2.onnx, already
                        running on the head cam → /yolo/detections)
open-vocab fallback   : GroundingDINO on RSL — for classes YOLO doesn't know
pseudo-labeler        : GroundingDINO + depth → generates pick data
manipulation          : π0 fine-tune owns the contact-rich grasp
navigation            : engineered encoder/Nav2 stack (NOT the VLA)
```

Operator, verbatim:

> "the full pipeline i was running for cleaningrobot before with the
> yolo3ddetector actually used the finetuned yolo for which i achieved a higher
> accuracy, i was not planning on making calls to rsl groundingdino — i only
> used that for pseudolabeling."

**So: DINO is scaffolding, not the destination.** A `yolo_trt_py` node instance
pointed at `/wrist_cam/image_raw` would give on-device wrist detection with no
tunnel dependency — this is a known-good, not-yet-done improvement. `socks2.plan`
(TensorRT engine) already exists.

The long-term product loop: wander → detect → approach → **π0 grasp** → carry →
place in basket, with the flywheel continuously distilling DINO pseudo-labels
into better on-device models.

---

## 3. What the teacher pipeline does (and its hard-won constants)

`pick_pipeline.py --wrist-only`: nominal hover (300,0) → wrist DINO detect →
iterative refine against a learned grasp anchor → sweep-in grasp → dual-camera
verify → optional `--drop-at`.

Constants that cost real debugging time — **do not "clean these up"**:

- Wrist px→mm Jacobian lives in `/demos/wrist_anchor.json`; corrections must be
  **rotated by base yaw** (`R(atan2(by,bx))`) because the camera rides the base
  servo while J was measured at yaw≈0.
- Grasp anchor `(243,251)` was learned by the robot itself (`--self-anchor`),
  replacing a stale human guess of `(382,342)`.
- Floor-grasp reach dies past **r≈340**; `send()` silently CLAMPS out-of-range
  commands, so an unguarded far target short-picks every single time. There is
  an explicit reach gate + in-reach-neighbor retarget + "needs a base move" skip.
- Grasp depth `gz`: scrunched sock −195, flattened −203, floor −212. Repeated
  pick-drop cycles UNROLL socks, so hold rate decays mid-session — look at the
  scene before blaming code.
- Verify thresholds: wrist held-check needs area **>60k px²** (a floor sock from
  a high pose subtends 15–20k and used to pass an 18k bar), overwhelming
  override **>120k**. Do **not** apply the claw dark-filter to the held-check —
  a held dark sock is precisely what it rejects.
- Depth is blind below ~300mm, which is why wrist-only mode exists at all: the
  head D455 cannot localize inside the arm's own workspace.

---

## 4. Recent changes (this session, all uncommitted)

### `scripts/robot` (NEW — the hardened entrypoint)
Modes table above. Mutex via `flock`, per-mode preflight, audit log.

### `scripts/pick_pipeline.py`
- **Parallax probe for claw self-detection.** A dark floor object next to the
  dark claw matched the "claw finger" filter and was invisible to the pipeline —
  the operator proved it by teleop-picking the sock the robot "couldn't see."
  Now: jog the hover 40mm and re-detect; boxes whose pixel *moves* are real
  floor objects (whitelisted), boxes that stay put are fingers.
- **Distractor filter** (`distractor_filter()`), replacing a naive
  confidence-threshold raise. Measured on a live frame: a metal creamer cup
  scored **0.42** while the real white sock scored **0.38** and the black sock
  **0.33** — DINO confidence is *inverted* here, so any threshold that kills the
  cup kills both socks first. Instead: minimum area (6k px²; the cup det was 3k)
  plus counter-prompt arbitration ("metal cup. mug. jar. bottle. plastic bag.")
  that rejects a box when the distractor prompt claims it more strongly.
  Fails **open** if the DINO tunnel is down.
- Earlier in the session: reach gate, neighbor-aware short sweep (stops the
  sweep from scooping a second object), verify threshold fixes, `--gz`,
  `--drop-at`, `--wrist-only`, `--self-anchor`, `--calibrate-wrist`.

### `src/robot_bringup/launch/robot_bringup.launch.py`
- `rosbridge_server` now has `respawn=True` — see §5.
- `flywheel_relay` added to the launch stack.

### `scripts/wrist_cam.py`
Dedicated reader thread + freeze watchdog. OpenCV/V4L2 timer-reads let frames
queue: full frame *rate* with seconds-stale *content*, which silently poisoned
refine. `refine()` additionally refuses to correct if the view didn't change
across the hover move.

### `scripts/flywheel_relay.py` (NEW)
1 Hz republisher of the last `/flywheel/{head,wrist}/overlay` + `/flywheel/meta`
so the console's flywheel panel isn't empty between runs.

### `src/robot_teleop/robot_teleop/arm_link.py`
Globs `/dev/ttyUSB*` on open — servo-rail brownouts re-enumerate the CP210x
(ttyUSB0 → ttyUSB1) and the fixed path broke the link.

---

## 5. Bugs found late, and what they teach

1. **`robot estop` was a NO-OP.** It published the bare string `estop` to
   `/teleop/cmd`, but that handler only parses JSON operator frames and silently
   drops anything else; the literal action lives on **`/teleop/action`**.
   Fixed. *Verify safety paths actually fire — never assume.* (The browser
   E-STOP was always fine.)
2. **rosbridge had no `respawn`.** It died and stayed dead; the console still
   served from :8080 and showed "no signal" indefinitely with nothing in any log.
   Fixed in the launch file **and** via the new `robot viz` for live repair.
3. **Killing a batch script is not a safe stop.** The in-container pipeline
   keeps its last commanded pose; if that pose is at grasp depth the servos hunt
   against the floor (operator observed violent shaking). **DONE 2026-08-16: `robot halt`** — disarms the π bridge (log-verified),
   SIGTERMs pipeline processes, lifts to safe pose unless estop is latched.
   `pick_pipeline.py` now traps SIGINT/SIGTERM into its own lift-then-exit.
4. **`docker exec` timeouts kill the client, not the process.** Unbounded
   `ros2 topic echo` leaked 8 orphaned subscribers. Bound with in-container
   `timeout`.
5. **A self-written evaluator lied.** A π0 "5/5" eval was fabricated by a
   holds-check run from an arbitrary pose, where a floor sock fills >60k px² and
   reads as "held" — the log even showed successes in trials where the *teacher*
   had failed to stage the sock. Retracted. **Any eval must command the standard
   high verify pose first, and must void trials where staging failed.**

---

## 6. π0 status — the honest version

- **Checkpoint:** `~/checkpoints/pi0_sock_v1/4999` on the **Jetson** (8.8 GB),
  rsynced to RSL at `~/checkpoints/pi0_sock_v1/4999`.
- **Serving:** on RSL, currently running:
  ```
  cd ~/openpi && uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_roarm_sock_cartesian_lora \
    --policy.dir=$HOME/checkpoints/pi0_sock_v1/4999
  ```
  Config injection (openpi commit `15a9616`): copy `roarm_policy.py` into
  `src/openpi/policies/`, append `roarm_inject_block.py` to
  `src/openpi/training/config.py`. Norm stats load from the checkpoint's own
  `assets/`. Health check: `curl :8000` returns **426** when healthy.
  ~348 ms/inference, returns (50, 4) cartesian chunks.
- **Behavior:** reaches toward the sock, hovers touching it, **does not close
  the grip**. 0 confirmed grasps in the current room. The July 2/3 result was in
  the *training* room. This is textbook distribution shift plus the freeze mode
  the fine-tune notes predicted for an 11-minute dataset with no recovery data.
- This fine-tune consumes the **head camera only**, so the wrist-mount rotation
  that broke the scripted pipeline is irrelevant to it.

**Note on terminology:** "LeRobot conversion" is not robot-specific — LeRobot is
the *dataset format* openpi trains from.
`openpi_roarm/convert_roarm_to_lerobot.py` already maps RoArm M2-S episodes
(frames + cartesian `[x,y,z,grip]`) into it; that exact path produced the
existing checkpoint.

---

## 7. Next steps, in priority order

1. ~~**`robot halt` mode**~~ DONE 2026-08-16 (see §5.3).
2. **Retrain π0 — now dual-camera.** The whole wrist path was wired
   2026-08-16: converter `--wrist --include-misses` (fixing the converter's
   silent miss-skipping, which contradicted this very plan), new config
   `pi0_roarm_sock_cartesian_wrist_lora` (repo `roarm_sock_cartesian_wrist`;
   a NEW name so the served v1 checkpoint keeps its exact inputs), policy
   fills `left_wrist_0_rgb` + mask, `pi_bridge` gained `wrist_topic` for
   serving (refuses to infer on a missing wrist frame — no silent
   train/serve mismatch). Rationale: the grasp happens in the head cam's
   depth-blind zone — v1 hovers without closing because it cannot SEE the
   grasp. On the GPU box:
   `./gpu_bootstrap.sh --wrist --include-misses --demos demos/picks --train`
   (~8 h / ~$4 on RunPod; needs neither the robot nor the operator).
   Coverage note: all 78 picks episodes have paired wrist frames except
   ep_0035 (wrist died mid-episode; auto-skipped). The 15 older demos/ep_*
   are head-only and excluded in wrist mode.
3. **Fix the evaluator**, then re-baseline π0 honestly (§5.5).
4. **Wrist YOLO**: second `yolo_trt_py` instance on `/wrist_cam/image_raw`;
   prefer it over DINO for socks in `refine()`. Removes the tunnel from the
   critical path and moots the whole creamer-vs-sock confidence problem.
5. **Commit this work** — 6 modified files + 4 new scripts are sitting
   uncommitted on `sock-demos-and-pi-finetune`.
6. When the base is powered: fish/flip-flop area + wicker basket →
   mobile pick→carry→place (tasks #24/#25).

---

## 7b. Changes 2026-08-16 (this session)

- `robot classic` mode + `docker/docker-compose.classic.yml`: sanctioned way to
  run the original YOLO 2D→depth→3D→arm_bridge autonomous pipeline (§ mode
  table). Compose override, never edits the base yaml; bring-up gates the loop.
- `robot halt` + `pick_pipeline` SIGINT/SIGTERM trap (§5.3 — DONE).
- Wrist-camera training path end to end (§7.2): converter/config/policy/bridge.
- Committed `6515805` (prior session's 15-file working tree) on
  `sock-demos-and-pi-finetune`; not yet pushed.

## 7c. Afternoon session 2026-08-16 (fusion + verify integrity)

- **Wrist+head socks2 on-device** (`yolo_trt_py`, contiguity + BGR fixes,
  stamp-gated freshness). DINO remains: counter-prompt arbitration (both
  detector paths -- the sock-filled plastic bag detects as sock THROUGH the
  plastic), head coarse, and pseudo-labeling.
- **head_scout fusion**: head 2D + floor-plane ray-cast -> arm targets inside
  the depth-blind band; aims wrist-only hovers (offline-validated via
  `scripts/scout_validate.py`, a no-motion harness that feeds the SUDS
  overlays). KNOWN GAP: plane fit often rejected at the OBSERVE pose (arm in
  the depth band despite side-column masking) -- needs an outlier-refit.
- **Near-end retarget** for elongated socks (centroid maps ~10cm past the
  graspable near end); FLOOR_REACH_R centralized, operator-reduced to 320.
- **VERIFY REBUILT** after five detector-based false successes (all audited,
  labels corrected in demos/picks): high pose first (5.5 doctrine), then
  claw-region pixel-diff vs an empty-claw session-start reference (empty ~7,
  held >14, overwhelming >40). Detector-free; the scene permanently contains
  a dark sock-shaped claw, so every detector referee eventually lies.
- **Depth-loft for auto-gz is impossible** in the pick zone (RealSense
  min-range ~300mm, measured); scout-aimed picks default gz -203.
- Autonomous holds today: anchor re-learn grasp, ep_0090, ep_0105
  (operator-confirmed). Wrist-frame claw grouping = socks2 OOD -> the
  pseudo-label/fine-tune with wrist frames is the durable detector fix.

## 7d. Batch certification 2026-08-16 (end of day)

5-episode unattended batch on the yolo+fusion pipeline: **2 held / 3
missed, all 5 labels frame-audited truthful** (holds: claw-diff 40.1 &
36.8 with the sock visibly clamped; misses: 5.5/3.1 empty + one honest
out-of-reach skip). hard_lock/soft-lock separation landed: the off-target
gate keys only on scout-measured targets, parallax self-locks are
sorting-only.

OPEN CALIBRATION ITEM: at the scout-aimed close hover (~r=195) the
wrist-implied det positions disagree with the scout target by a
CONSISTENT 168-186mm across every attempt -- picks succeed via the scan
hovers instead. Systematic, not noise; suspect anchor/Jacobian validity
at that close geometry. Re-run `robot calibrate` / self-anchor from a
~195mm hover, or map px->mm scale vs hover radius.

## 8. Operating notes / gotchas

- Config truth lives in `docker/docker-compose.yml` env vars, **not** the launch
  file's `DeclareLaunchArgument` defaults (those are stale).
- DINO tunnel:
  `ssh -f -N -L 8002:localhost:8002 -L 8000:localhost:8000 -o HostKeyAlias=RSL aaron@100.110.113.91`
  It has died mid-batch more than once; `refine()` now prints
  `[refine] DINO unreachable` loudly instead of swallowing it as "no detection."
- Episodes under `demos/` are root-owned (written from the container) — delete
  via the container, not the host.
- Optical flow (Farneback) is **useless** on the textureless wood floor; it
  returned ~zero on real motion and misled debugging twice. Use DINO tracking.
- The RoArm firmware **echoes IK pose even with servos unpowered** — encoder
  feedback changing does NOT prove physical motion. Only vision or the operator
  confirms movement.
- `pgrep -f` inside a `docker exec bash -c` matches its own command line; use
  the `[r]osbridge` bracket trick.

## 9. Standing operator constraints

- Never print the contents of `~/.runpod_key` or `~/.dino_key`.
- Do not store anything about Railway (operator asked for it to be forgotten).
- The leaked `id_ed25519` is treated as compromised.
- Base motion requires an explicit operator grant.
- **Dataset labels must stay truthful** — a miss recorded as a success poisons
  the fine-tune. This is the one class of mistake that is never acceptable;
  false-positive grasps and wasted cycles are fine and even useful.
