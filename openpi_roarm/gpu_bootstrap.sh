#!/usr/bin/env bash
# One-shot setup for the RoArm sock π0 fine-tune, run ON A FRESH GPU BOX.
#
# Does everything mechanical: clone openpi, install, inject our policy+config,
# build the LeRobot dataset from demos/, compute norm stats (which also
# integration-tests the whole config+data path), and print the exact train +
# serve commands. It deliberately STOPS before the multi-hour train() so you can
# eyeball that setup succeeded first.
#
# Prereqs on the box: an NVIDIA GPU (>=24 GB for the LoRA config), git, curl,
# and the two payload files from this repo's openpi_roarm/ next to this script:
#   roarm_policy.py, roarm_inject_block.py
# plus the demos (either a demos/ dir or demos.tar next to this script).
#
# Usage:
#   ./gpu_bootstrap.sh [--space cartesian|joint] [--demos ./demos ...] [--train]
#                      [--wrist] [--include-misses]
#
# --train also kicks off training at the end instead of just printing the cmd.
# --wrist trains the dual-camera config (pi0_roarm_sock_cartesian_wrist_lora):
#   head + wrist views; episodes without paired frames_wrist/ are skipped.
# --include-misses keeps success=false episodes (recovery data).
# --demos may be given more than once to combine roots (e.g. demos demos/picks).
set -euo pipefail
# Explicit failure marker: monitors must grep for "SETUP OK" / "BOOTSTRAP FAILED"
# rather than inferring health from process liveness (which lies).
trap 'echo "BOOTSTRAP FAILED (line $LINENO: $BASH_COMMAND)"' ERR

SPACE=cartesian
DEMOS=()
DO_TRAIN=0
WRIST=0
MISSES=0
HERE="$(cd "$(dirname "$0")" && pwd)"
while [ $# -gt 0 ]; do
  case "$1" in
    --space) SPACE="$2"; shift 2;;
    --demos) DEMOS+=("$2"); shift 2;;
    --train) DO_TRAIN=1; shift;;
    --wrist) WRIST=1; shift;;
    --include-misses) MISSES=1; shift;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done
[ ${#DEMOS[@]} -gt 0 ] || DEMOS=(./demos)
REPO_ID="roarm_sock_${SPACE}"
# config names must match those built in roarm_inject_block.py
if [ "$WRIST" = 1 ]; then
  [ "$SPACE" = cartesian ] || { echo "--wrist config exists for cartesian only"; exit 2; }
  REPO_ID="roarm_sock_cartesian_wrist"
  CONFIG="pi0_roarm_sock_cartesian_wrist_lora"
elif [ "$SPACE" = joint ]; then
  CONFIG="pi0_roarm_sock_lora"
else
  CONFIG="pi0_roarm_sock_cartesian_lora"
fi
CONVERT_FLAGS=()
[ "$WRIST" = 1 ] && CONVERT_FLAGS+=(--wrist)
[ "$MISSES" = 1 ] && CONVERT_FLAGS+=(--include-misses)

say() { printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }

# 0. sanity
if ! command -v nvidia-smi >/dev/null; then
  echo "no nvidia-smi -- is this a GPU box?"; exit 1
fi
nvidia-smi -L
[ -f "$HERE/roarm_policy.py" ] || { echo "missing roarm_policy.py next to script"; exit 1; }
[ -f "$HERE/roarm_inject_block.py" ] || { echo "missing roarm_inject_block.py"; exit 1; }

# Locate the demos. Accept existing dirs, else unpack demos.tar from any of
# the places it plausibly landed (beside the script, the parent dir -- e.g.
# `scp -r openpi_roarm demos.tar host:~/` puts them as siblings -- or $PWD).
if [ ! -d "${DEMOS[0]}" ] && [ ${#DEMOS[@]} -eq 1 ]; then
  for d in "$HERE" "$HERE/.." "$PWD"; do
    if [ -d "$d/demos" ]; then DEMOS=("$d/demos"); break; fi
    if [ -f "$d/demos.tar" ]; then
      say "unpacking $d/demos.tar"
      # --no-same-owner: network volumes (e.g. RunPod /workspace on MooseFS)
      # refuse chown even for root, and a failed chown fails tar entirely.
      tar --no-same-owner -xf "$d/demos.tar" -C "$d"
      DEMOS=("$d/demos"); break
    fi
  done
fi
N_EPS=0
for root in "${DEMOS[@]}"; do
  if [ ! -d "$root" ]; then
    echo "demos root not found: $root. Put demos.tar (or a demos/ dir) beside"
    echo "this script, in its parent dir, or pass --demos /path/to/demos"; exit 1
  fi
  N_EPS=$((N_EPS + $(ls -d "$root"/ep_* 2>/dev/null | wc -l)))
done
echo "demos: $N_EPS episodes across ${#DEMOS[@]} root(s)"

# 1. openpi
if [ ! -d "$HOME/openpi" ]; then
  say "cloning openpi"
  git clone https://github.com/Physical-Intelligence/openpi "$HOME/openpi"
fi
cd "$HOME/openpi"
command -v uv >/dev/null || { say "installing uv"; curl -LsSf https://astral.sh/uv/install.sh | sh; export PATH="$HOME/.local/bin:$PATH"; }
say "uv sync (this pulls torch/jax; slow first time)"
GIT_LFS_SKIP_SMUDGE=1 uv sync

# 2. inject our policy + config
say "injecting roarm policy + config"
cp "$HERE/roarm_policy.py" src/openpi/policies/roarm_policy.py
CFG=src/openpi/training/config.py
if ! grep -q "RoArm sock fine-tune (auto-injected" "$CFG"; then
  cat "$HERE/roarm_inject_block.py" >> "$CFG"
  echo "appended config block to $CFG"
else
  echo "config block already present; skipping"
fi

# 3. build the LeRobot dataset (skip if a previous run already built it)
DS_DIR="$HOME/.cache/huggingface/lerobot/$REPO_ID"
if [ -d "$DS_DIR/meta" ]; then
  say "dataset $REPO_ID already exists at $DS_DIR -- skipping conversion"
else
  say "building LeRobot dataset ($SPACE space) -> $REPO_ID"
  uv run python "$HERE/convert_roarm_to_lerobot.py" \
      --demos "${DEMOS[@]}" --repo-id "$REPO_ID" --space "$SPACE" \
      --prompt "pick up the sock" \
      ${CONVERT_FLAGS[@]+"${CONVERT_FLAGS[@]}"}
fi

# 4. norm stats  (LOADS the config + dataset end to end == integration test)
say "compute_norm_stats ($CONFIG) -- this validates the whole config/data path"
# tyro exposes the arg as either positional or --config-name depending on version
uv run scripts/compute_norm_stats.py "$CONFIG" \
  || uv run scripts/compute_norm_stats.py --config-name "$CONFIG"

EXP=sock_v1
[ "$WRIST" = 1 ] && EXP=sock_v2_wrist
say "SETUP OK"
cat <<EOF

Next:
  cd ~/openpi
  # train (LoRA, ~30k steps; watch the first ~200 steps for a falling loss):
  uv run scripts/train.py $CONFIG --exp-name $EXP
  # then serve the checkpoint:
  uv run scripts/serve_policy.py --config $CONFIG \\
      --checkpoint checkpoints/$CONFIG/$EXP/<step>

EOF

if [ "$DO_TRAIN" = 1 ]; then
  say "starting training ($CONFIG)"
  uv run scripts/train.py "$CONFIG" --exp-name "$EXP"
fi
