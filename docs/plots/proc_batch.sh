#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Evaluates and generates plots for all bags

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/../../scripts/common.sh"
source "$SCRIPT_DIR/../../.venv/bin/activate"

printInfo "Evaluating all bags..."

cd "$SCRIPT_DIR"
for d in ../../bags/*/; do
    bag=$(basename "$d")
    printInfo "Selecting ${bag}..."
    ./evo_eval.sh "$bag"
done

printInfo "Generating plots..."

python3 traj_plot.py
python3 metrics_plot.py
