# RoArm sock pick → π0 fine-tune

Everything needed to fine-tune Physical Intelligence's π0 on our 50 RoArm sock
demos and drive the arm with the result.

## ⚡ Minimum steps you have to do

Everything else is automated by `gpu_bootstrap.sh`.

**1. Rent a GPU box** (I can't — needs an account + payment). You need
**≥24 GB VRAM**; 40–48 GB is comfortable. Any of:
- **RunPod** (easiest, per-second billing) — an **A40 48 GB** or **L40S 48 GB**
  is the sweet spot, roughly **$0.40–0.90/hr** at time of writing.
- **Lambda Labs** — A100 40 GB, ~$1.30/hr.
- **Vast.ai** — cheapest/spot, an RTX 4090 24 GB will just fit LoRA.

Pick a **PyTorch/CUDA** template, ≥100 GB disk (checkpoints + dataset are big).

**2. Copy two things up** (from the robot):
```bash
scp -r openpi_roarm demos.tar  <user>@<gpu-box>:~/
```
(`demos.tar` is 700 MB, sitting in the repo root; regenerate with
`tar -cf demos.tar demos/`.)

**3. Run one command** on the box:
```bash
cd ~/openpi_roarm && ./gpu_bootstrap.sh --space cartesian
```
That clones openpi, installs it, injects our policy + config, unpacks the demos,
builds the LeRobot dataset, and runs `compute_norm_stats` — which loads the whole
config+data path, so if it finishes you know the integration works. It stops
before training and prints the exact train command.

**4. Train** (it prints this for you):
```bash
cd ~/openpi && uv run scripts/train.py pi0_roarm_sock_cartesian_lora --exp-name sock_v1
```
⚠️ **Lower `num_train_steps` to ~5000 for the first run** (edit the injected
block at the bottom of `src/openpi/training/config.py`). 30k steps on only 50
episodes will overfit and costs ~10–20 h of GPU time; 5k is a few hours and
tells you whether it's learning. Watch that the loss falls over the first few
hundred steps.

**5. Serve it**, then point the robot at it (see "Driving the arm" below):
```bash
uv run scripts/serve_policy.py --config pi0_roarm_sock_cartesian_lora \
    --checkpoint checkpoints/pi0_roarm_sock_cartesian_lora/sock_v1/<step>
```

## Files here

| File | Runs on | What it is |
|---|---|---|
| `gpu_bootstrap.sh` | GPU box | One-shot: clone → install → inject → dataset → norm stats |
| `convert_roarm_to_lerobot.py` | GPU box | Builds a real LeRobot dataset from `demos/` via the `lerobot` API |
| `roarm_policy.py` | GPU box (into openpi) | Transforms: our 4-DoF + 1 cam ↔ pi0's padded 3-cam tensors |
| `roarm_inject_block.py` | GPU box (into openpi) | `RoarmDataConfig` + the two `TrainConfig`s, appended to `config.py` |

## Why fine-tune at all (the domain gap)

The model on `RSL:8000` is **π0.5-DROID**: 7-DoF Franka, 8-dim **joint-velocity**
actions, exterior+wrist cameras. Ours is a **4-DoF** RoArm, **position** control,
**one** camera. `scripts/pi_bridge.py` currently pokes that server with **zeroed
proprioception** — a smoke test, not control. Zero-shot DROID can't drive our
arm. Hence a LoRA fine-tune of **π0 base** on our own data.

## The data

**50 successful demos**, 6808 frames, `demos/ep_0000..0049` — every pick
torque-verified, no failures recorded. Task: `"pick up the sock"`.

Two possible spaces (pick one; `--space` selects it):
- **cartesian** `[x, y, z, gripper]` — **recommended**: policy output maps 1:1
  to a `T:1041` arm command, no forward kinematics.
- **joint** `[base, shoulder, elbow, gripper]` — the arm's own feedback.

50 episodes proves the task but is thin. Plan on 100+ across varied sock shapes,
positions and lighting before trusting it — collect more with
`scripts/sock_cycle.py`.

## Driving the arm with it (pi_bridge rework)

`scripts/pi_bridge.py` changes from the DROID smoke-test to our real policy:

- **Send real observation**, not zeros — camera + current pose from
  `/teleop/state`:
  ```python
  obs = {
      "observation/image": resize_with_pad(rgb, 224, 224),
      "observation/state": np.array([x, y, z, grip], np.float32),
      "prompt": prompt,
  }
  ```
- **Expect 4-dim actions** back, not 8.
- **Execute** each action as a `T:1041` setpoint by publishing `goto:x,y,z,t` on
  `/teleop/action` — the same channel `sock_cycle.py` uses, so `teleop_node`
  stays the single serial owner and the browser E-STOP still works. Stream the
  chunk at the ~15–20 Hz the demos were recorded at.

## If the injected config doesn't load

`roarm_inject_block.py` mirrors openpi's own LIBERO example. If their API has
moved, two lines are flagged `VERSION-SENSITIVE` (`create_base_config(...)` and
`ModelTransformFactory()(...)`) — compare against the libero config in
`src/openpi/training/config.py` and match it. Everything else is generic.
