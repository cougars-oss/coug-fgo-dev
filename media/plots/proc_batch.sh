#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Evaluates and generates plots for all bags
#
# Usage:
#   ./proc_batch.sh [evo_args...]
#
# Arguments:
#   [evo_args...]: Additional arguments passed directly to evo_eval.sh -> evo_ape/evo_rpe
#
# Common Evo Arguments:
#   --align: Align trajectories using Umeyama's method (best fit)
#   --align_origin: Align trajectories using the first pose (origin)
#   --project_to_plane xy: Project trajectories to the 2D plane (xy)
#   --n_to_align <N>: Number of poses to use for alignment

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/../../scripts/common.sh"
source "$SCRIPT_DIR/../../.venv/bin/activate"

printInfo "Evaluating all bags..."

cd "$SCRIPT_DIR"
for d in ../../bags/*/; do
    bag=$(basename "$d")
    printInfo "Selecting ${bag}..."
    ./evo_eval.sh "$bag" "$@"
done

printInfo "Generating plots..."

python3 traj_plot.py
python3 metrics_plot.py
