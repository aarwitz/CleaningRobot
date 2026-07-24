# Fine-tuning π0 on a $200 arm: end-to-end findings

*A complete record of taking Physical Intelligence's π0 from "cloned the repo"
to "the network picked up a sock on my living-room table," on hobby hardware,
in one session. Written 2026-07-24.*

## Executive summary

We fine-tuned **π0 (base)** with LoRA on **50 self-collected demonstrations
(11 minutes of robot time)** of a sock pick-and-place, and deployed it
closed-loop on the robot that collected them. The policy **successfully picked
up the sock in 2 of 3 live attempts** (verified by gripper-torque telemetry and
witnessed), reproducing the demonstrated grasp strategy — a one-sided-claw
lateral sweep — from a single RGB camera, 4 floats of proprioception, and a
text prompt. Every observed failure traced to a specific, fixable property of
our data, not to the model. Total marginal cost: **~$5 of cloud GPU**.

The most transferable finding: **data curation choices dominate everything
else.** We deleted failed picks to keep the dataset "clean" — and thereby
built a policy that had never seen the state following a miss. It froze,
mode-averaging, precisely at those states. PI's practice of keeping
corrections/recoveries in demonstration data is not a nicety; it is the
difference between a demo and a skill.

## The robot

| Component | Detail |
|---|---|
| Base | Custom mecanum mobile robot, Jetson (aarch64), ROS 2 Humble in an Isaac ROS container |
| Arm | Waveshare RoArm M2-S, 4-DoF, **one-sided claw** (single moving jaw), $~200 class |
| Camera | RealSense D455, RGB only for the policy (640×480 → 224×224) |
| Proprioception | The arm's own serial feedback: cartesian x, y, z (mm) + gripper angle (rad) |
| Compute (train) | RunPod A40 48 GB, $0.44/hr |
| Compute (infer) | Same A40, served over websocket through an SSH tunnel to the robot |

Constraints that shaped everything: the camera rides the arm's rotating base
(the gripper is nearly fixed in the image while the world pans); the claw is
one-sided so a naive top-down pinch fails on compressible objects; the D455
returns no depth below 0.30 m (irrelevant to the policy — it never sees depth —
but it shaped the scripted collection).

## Data collection (the part that mattered most)

`scripts/sock_cycle.py` — a scripted collector that owns **no hardware**: it
drives the arm purely over ROS topics through the same teleop node the human
operator uses, so the browser E-STOP halts scripted collection too, and the
firmware-wedging serial command (`T:104`, blocking) is never used.

**The grasp had to be engineered before it could be demonstrated.** A top-down
pinch on a scrunched sock measurably fails: the servo stalls on a fold
(commanded−measured gap 0.28 rad) which extrudes out during the lift
(gap → 0.01, drop). The working strategy — discovered with the operator over
several iterations — is a **sweep-in**: descend beside the sock on the moving-
jaw side, fold the last 25 mm of descent into a lateral sweep, and close the
claw *during* the sweep (finishing before the lateral motion ends) so the
moving jaw drags the sock into the fixed jaw. Grasp verification is
**gripper torque** (|torH| ≥ 50 ⇒ held; empty close reads ~32) plus the
commanded-vs-measured claw gap, checked after the close *and* after the lift.

Numbers:
- **50 episodes, 6,808 frames, 0 failures recorded** (torque-verified, misses
  auto-discarded — see "the QC mistake" below)
- ~13 s/episode; **~11 minutes total robot time**
- Each frame: RGB image, cartesian state [x,y,z,grip], 3 joint angles,
  4 servo torques, and the commanded target
- Ground truth chains forward with no perception: each episode's (jittered)
  place point is the next episode's pick point, bounded to ±32 mm so the random
  walk cannot leave the workspace

## Training pipeline

- **Conversion**: raw episodes → LeRobot v2 via the `lerobot` API on the GPU
  box. State/action = **cartesian [x, y, z, gripper]** — chosen so a policy
  action maps 1:1 onto an arm command (`T:1041` setpoint) with no forward
  kinematics. Action = next-step state (position control).
- **Config**: `Pi0Config` with LoRA variants (`gemma_2b_lora` +
  `gemma_300m_lora`), starting from the released `pi0_base` checkpoint.
  Following the LIBERO recipe for absolute-action datasets:
  **DeltaActions on x,y,z; gripper stays absolute** (π0 is pretrained on
  deltas). One integration gotcha: openpi's `DataConfig.action_sequence_keys`
  defaults to the libero-style column name `("actions",)`; standard LeRobot
  datasets use `action`.
- **Run**: 5,000 steps, batch default, `WANDB_MODE=offline`, single A40.
  **6.1 s/step ⇒ 8h15m wall, ~$3.90.** First-request serving JIT ≈ tens of
  minutes on a small CPU slice (cached thereafter); steady-state inference
  **~330–480 ms** per 50-step chunk.

## What the policy actually receives and emits

Per inference (~0.5 s round trip, robot → A40 → robot):

**In:** one 224×224 RGB frame (letterboxed from 640×480) · 4 floats
`[x, y, z, grip]` from the arm's real feedback · the string
`"pick up the sock"`. Nothing else — no depth, no detector, no history.

**Out:** a **50×4 action chunk** ≈ 3.3 s of motion at the 15 Hz demo rate:
x,y,z deltas (decoded to absolute against current state) + absolute gripper.

**Execution:** every action becomes a `goto:` setpoint on the same topic the
human teleop uses — envelope-clamped and E-STOP-gated by the teleop node,
which remains the only process touching the serial port. During policy runs,
the bridge is the *only* publisher of arm commands (verified via topic
introspection and a full rosbag of the run: 601 commands, 15 inferences,
5,038 camera frames — `demos/pi_demo_bag`).

## Live results

| Attempt | Start state | Outcome |
|---|---|---|
| Dry-run (no motion) | arm at trained hover | Chunk reproduces the trained sweep: descend from +y, close 1.5→2.7 mid-sweep, bottom at z −117 (trained −121), endpoint on the sock's true position |
| 1 | clean hover, tight wad | Near-grasp at t+5s (torH 64), closed ~30 mm high, fumbled, displaced sock |
| 2 | clean hover, tight wad | (Ran un-commanded — see incident) **Picked and cycled pick/place repeatedly**, witnessed |
| 3 (recorded) | clean hover, tight wad | **Clean pick at t+3s** (torH −116 sustained), lift holding, then released ~40 mm high, re-descended, froze; auto-disarm ended the run |

**Success signature** (real hold): claw ~3.0 with sustained |torH| 150–240.
**Failure signature** (the important one): position frozen while the gripper
oscillates open/closed. This is behavior-cloning **mode averaging** made
visible by our action space: x,y,z are *delta* actions, so indecision between
"descend" and "lift" averages to ~zero motion — the arm freezes; the gripper
is *absolute*, so the same indecision oscillates between the open-mode and
close-mode values. One trace, two encodings, both failure modes legible.

## The five lessons

**1. Pretrained VLA generality is real and cheap to tap.** 11 minutes of
demonstrations + $4 of LoRA produced a working visuomotor pick on an
embodiment π0 never saw — a 4-DoF hobby arm. The base model contributed
everything we didn't have data for: visual robustness (it survived furniture
changes, lighting shift, and a guitar appearing in frame), smooth reaching,
the concept of "grasp the named object."

**2. Zero-shot cross-embodiment does not work; fine-tuning bridges it.** The
stock π0.5-DROID checkpoint (7-DoF Franka, joint-velocity, dual camera)
cannot drive this arm — the action semantics don't map. The entire gap closed
with a config: pad 4-DoF state into the model's 32-dim action space, mask the
two missing cameras, slice 4 dims back out of the output.

**3. Never curate away the failures.** Our collector deleted every missed
grasp (torque-verified) to keep training data "clean." Consequence: zero
recovery examples, and a policy that visits a post-miss state has no mode to
express — it freezes. PI's inclusion of corrections in demo data is
load-bearing. The fix is one flag: record the scripted miss-and-retry
sequences too.

**4. Respect the chunk.** The demos are effectively open-loop 50-step motion
programs. Executing 25 steps then re-planning injected mid-descent state
jitter the training never saw — the policy closed early (z −85 vs trained
−121) in both fumbled attempts. Executing full 50-step chunks matched the
data's temporal structure; both successful picks happened on coherent first
chunks.

**5. Verify state; never trust your own success message.** Three separate
incidents in one project, same root cause:
   - a monitoring `pgrep` matched its own command line and reported a dead
     bootstrap as alive;
   - a training launch failed instantly (`uv` not on PATH) under a monitor
     that would not have noticed for 8 hours;
   - a "STOP SENT" echo printed unconditionally while the stop had failed —
     the policy kept running the arm and *nobody knew* until the operator saw
     the arm lift the sock un-commanded.
   The pattern that fixed all three: processes emit explicit outcome markers
   (`TRAIN_OK` / `BOOTSTRAP FAILED` / logged `request→mode` transitions),
   monitors grep markers, and every mode change is verified by reading state
   back. On robots this is not software hygiene; it is safety.

## How this maps to PI's intended training recipe

π0's two layers answer the "how am I supposed to train this?" question:

- **Base (theirs):** ~10k hours of cross-robot, cross-home teleop. This is the
  "drive around the apartment all day" layer — already in the checkpoint.
- **Fine-tune (yours):** 1–20 *hours* of targeted demonstrations of your task
  on your robot, *including corrections*. Our 11 minutes sits two orders of
  magnitude below the recommended floor — and still produced picks, which is
  the strongest available evidence for the base layer's value.

For a home-cleaning robot, the pragmatic architecture short-term is hybrid:
engineered navigation (our encoder/Nav2 stack) finds and approaches objects;
the VLA owns the contact-rich manipulation segment. Full end-to-end
fetch-and-clean is a data problem (mobile-manipulation demos with the base in
the action space), not a modeling problem.

## Next experiment (queued)

200–300 episodes (~1.5 h robot time, hands-off) with: miss-and-retry episodes
kept, position variety across the full reachable band, 2–3 sock types, both
lighting conditions, varied start states, deliberate slow release at the
surface. Same pipeline end-to-end; ~8 h/$4 to retrain. Predicted outcome:
the freeze disappears (recovery data), place height tightens, and success
becomes repeatable rather than probable.

## Artifacts

| What | Where |
|---|---|
| Collector, bridge, smoke test | `scripts/sock_cycle.py`, `scripts/pi_bridge.py`, `scripts/pi_smoke_test.py` |
| GPU pipeline (bootstrap, converter, transforms, config) | `openpi_roarm/` |
| Raw demonstrations | `demos/ep_0000..0049` (6,808 frames + trajectories) |
| Full recorded policy run | `demos/pi_demo_bag` (rosbag: all commands, inferences, frames) |
| Fine-tuned checkpoint | pod `/workspace/openpi/checkpoints/pi0_roarm_sock_cartesian_lora/sock_v1/4999` |
| Training log | pod `/workspace/train.log` (5k steps, 8h15m) |
