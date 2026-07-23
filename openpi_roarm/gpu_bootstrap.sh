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
#   ./gpu_bootstrap.sh [--space cartesian|joint] [--demos ./demos] [--train]
#
# --train also kicks off training at the end instead of just printing the cmd.
set -euo pipefail

SPACE=cartesian
DEMOS=./demos
DO_TRAIN=0
HERE="$(cd "$(dirname "$0")" && pwd)"
while [ $# -gt 0 ]; do
  case "$1" in
    --space) SPACE="$2"; shift 2;;
    --demos) DEMOS="$2"; shift 2;;
    --train) DO_TRAIN=1; shift;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done
REPO_ID="roarm_sock_${SPACE}"
# config names must match those built in roarm_inject_block.py
if [ "$SPACE" = joint ]; then
  CONFIG="pi0_roarm_sock_lora"
else
  CONFIG="pi0_roarm_sock_cartesian_lora"
fi

say() { printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }

# 0. sanity
if ! command -v nvidia-smi >/dev/null; then
  echo "no nvidia-smi -- is this a GPU box?"; exit 1
fi
nvidia-smi -L
[ -f "$HERE/roarm_policy.py" ] || { echo "missing roarm_policy.py next to script"; exit 1; }
[ -f "$HERE/roarm_inject_block.py" ] || { echo "missing roarm_inject_block.py"; exit 1; }

# Locate the demos. Accept an existing dir, else unpack demos.tar from any of
# the places it plausibly landed (beside the script, the parent dir -- e.g.
# `scp -r openpi_roarm demos.tar host:~/` puts them as siblings -- or $PWD).
if [ ! -d "$DEMOS" ]; then
  for d in "$HERE" "$HERE/.." "$PWD"; do
    if [ -d "$d/demos" ]; then DEMOS="$d/demos"; break; fi
    if [ -f "$d/demos.tar" ]; then
      say "unpacking $d/demos.tar"
      tar -xf "$d/demos.tar" -C "$d"
      DEMOS="$d/demos"; break
    fi
  done
fi
if [ ! -d "$DEMOS" ]; then
  echo "no demos found. Put demos.tar (or a demos/ dir) beside this script,"
  echo "in its parent dir, or pass --demos /path/to/demos"; exit 1
fi
echo "demos: $(ls -d "$DEMOS"/ep_* | wc -l) episodes"

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

# 3. build the LeRobot dataset
say "building LeRobot dataset ($SPACE space) -> $REPO_ID"
uv run python "$HERE/convert_roarm_to_lerobot.py" \
    --demos "$DEMOS" --repo-id "$REPO_ID" --space "$SPACE" \
    --prompt "pick up the sock"

# 4. norm stats  (LOADS the config + dataset end to end == integration test)
say "compute_norm_stats ($CONFIG) -- this validates the whole config/data path"
uv run scripts/compute_norm_stats.py "$CONFIG"

say "SETUP OK"
cat <<EOF

Next:
  cd ~/openpi
  # train (LoRA, ~30k steps; watch the first ~200 steps for a falling loss):
  uv run scripts/train.py $CONFIG --exp-name sock_v1
  # then serve the checkpoint:
  uv run scripts/serve_policy.py --config $CONFIG \\
      --checkpoint checkpoints/$CONFIG/sock_v1/<step>

EOF

if [ "$DO_TRAIN" = 1 ]; then
  say "starting training ($CONFIG)"
  uv run scripts/train.py "$CONFIG" --exp-name sock_v1
fi
