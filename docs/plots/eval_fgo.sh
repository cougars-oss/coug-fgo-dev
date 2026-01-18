#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Generates APE/RPE metrics for FGO evaluation
#
# Usage:
#   ./eval_fgo.sh <bag_name>
#
# Arguments:
#   <bag_name>: Evaluate ../../bags/<bag_name>

set -e

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/../../scripts/common.sh"
source "$SCRIPT_DIR/../../.venv/bin/activate"

evo_config reset

if [ "$#" -ne 1 ]; then
    printError "Usage: $0 <bag_name>"
    exit 1
fi

BAG_NAME="$1"
BAG_PATH="$SCRIPT_DIR/../../bags/$BAG_NAME"

if [ ! -d "$BAG_PATH" ]; then
    printError "Bag directory not found: $BAG_PATH"
    exit 1
fi

AGENTS=("auv0" "auv1" "auv2")
SUFFIXES=("odometry/global" "odometry/global_tm" "odometry/global_ekf" "odometry/global_ukf")
LABELS=("fgo" "tm" "ekf" "ukf")
METRICS=("trans_part" "angle_deg")

for agent in "${AGENTS[@]}"; do
    printInfo "Processing $agent..."
    TRUTH="/${agent}/odometry/truth"
    
    EST_TOPICS=()
    for s in "${SUFFIXES[@]}"; do EST_TOPICS+=("/${agent}/${s}"); done

    for i in "${!EST_TOPICS[@]}"; do
        printInfo "Analyzing ${LABELS[$i]} ($agent)..."
        TOPIC="${EST_TOPICS[$i]}"
        LABEL="${LABELS[$i]}"
        
        for metric in "${METRICS[@]}"; do
            mkdir -p "$BAG_PATH/${agent}/${LABEL}"

            # 1. APE (Global Accuracy)
            evo_ape bag2 "$BAG_PATH" "$TRUTH" "$TOPIC" -r "$metric" \
                --save_results "$BAG_PATH/${agent}/${LABEL}/ape_${metric}.zip"

            # 2. RPE (Drift - Normalized per 1 meter)
            evo_rpe bag2 "$BAG_PATH" "$TRUTH" "$TOPIC" -r "$metric" \
                --delta 1 --delta_unit m --all_pairs \
                --save_results "$BAG_PATH/${agent}/${LABEL}/rpe_${metric}.zip"
        done
    done
    
    printInfo "Exporting metrics ($agent)..."
    evo_res "$BAG_PATH/${agent}"/*/ape_trans_part.zip --save_table "$BAG_PATH/${agent}/metrics_ape_trans.csv"
    evo_res "$BAG_PATH/${agent}"/*/ape_angle_deg.zip  --save_table "$BAG_PATH/${agent}/metrics_ape_rot.csv"
    evo_res "$BAG_PATH/${agent}"/*/rpe_trans_part.zip --save_table "$BAG_PATH/${agent}/metrics_rpe_trans.csv"
    evo_res "$BAG_PATH/${agent}"/*/rpe_angle_deg.zip  --save_table "$BAG_PATH/${agent}/metrics_rpe_rot.csv"
done
