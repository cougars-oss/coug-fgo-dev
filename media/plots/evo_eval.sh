#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Generates APE/RPE benchmark metrics for FGO evaluation
#
# Usage:
#   ./evo_eval.sh <bag_name> [evo_args...]
#
# Arguments:
#   <bag_name>: Evaluate ../../bags/<bag_name> (required)
#   [evo_args...]: Additional arguments passed directly to evo_ape/evo_rpe
#
# Common Evo Arguments:
#   --align: Align trajectories using Umeyama's method (best fit)
#   --align_origin: Align trajectories using the first pose (origin)
#   --project_to_plane xy: Project trajectories to the 2D plane (xy)
#   --n_to_align <N>: Number of poses to use for alignment

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/../../scripts/common.sh"

if [ -z "$1" ]; then
    printError "Usage: $0 <bag_name> [evo_args...]"
    exit 1
fi

BAG_NAME="$1"
BAG_PATH="$SCRIPT_DIR/../../bags/$BAG_NAME"
shift

if [ ! -d "$BAG_PATH" ]; then
    printError "Bag directory not found: $BAG_PATH"
    exit 1
fi

METADATA_FILE="$BAG_PATH/metadata.yaml"
if [ ! -f "$METADATA_FILE" ]; then
    printError "Metadata file not found: $METADATA_FILE"
    exit 1
fi

evo_config reset &>/dev/null
evo_config set save_traj_in_zip true &>/dev/null

AGENTS=("auv0" "auv1" "auv2" "bluerov2")
SUFFIXES=("odometry/global" "odometry/global_tm" \
          "odometry/global_ekf" "odometry/global_ukf" \
          "odometry/global_iekf" "odometry/dvl")
LABELS=("fgo" "tm" "ekf" "ukf" "iekf" "dvl")
METRICS=("trans_part" "angle_deg")

for agent in "${AGENTS[@]}"; do
    TRUTH="/${agent}/odometry/truth"
    
    if ! grep -q "name: $TRUTH" "$METADATA_FILE"; then
        printWarning "Agent $agent not found, skipping..."
        continue
    fi

    printInfo "Processing $agent..."
    
    EST_TOPICS=()
    for s in "${SUFFIXES[@]}"; do EST_TOPICS+=("/${agent}/${s}"); done

    for i in "${!EST_TOPICS[@]}"; do
        TOPIC="${EST_TOPICS[$i]}"
        LABEL="${LABELS[$i]}"
        
        if ! grep -q "name: $TOPIC" "$METADATA_FILE"; then
            printWarning "Topic $TOPIC not found, skipping..."
            continue
        fi

        printInfo "Evaluating ${agent}/${LABEL}..."
        
        for metric in "${METRICS[@]}"; do
            mkdir -p "$BAG_PATH/evo/${agent}/${LABEL}"

            # 1. APE (Global Accuracy)
            if [ ! -f "$BAG_PATH/evo/${agent}/${LABEL}/ape_${metric}.zip" ]; then
                evo_ape bag2 "$BAG_PATH" "$TRUTH" "$TOPIC" -r "$metric" "$@" \
                    --save_results "$BAG_PATH/evo/${agent}/${LABEL}/ape_${metric}.zip"
            else
                printWarning "Skipping APE ${metric} for ${agent}/${LABEL} (already exists)"
            fi

            # 2. RPE (Drift - Normalized per 1 meter)
            if [ ! -f "$BAG_PATH/evo/${agent}/${LABEL}/rpe_${metric}.zip" ]; then
                evo_rpe bag2 "$BAG_PATH" "$TRUTH" "$TOPIC" -r "$metric" "$@" \
                    --delta 1 --delta_unit m --all_pairs \
                    --save_results "$BAG_PATH/evo/${agent}/${LABEL}/rpe_${metric}.zip"
            else
                printWarning "Skipping RPE ${metric} for ${agent}/${LABEL} (already exists)"
            fi
        done
    done
    
    printInfo "Exporting ${agent} metrics..."
    if [ ! -f "$BAG_PATH/evo/${agent}/metrics_ape_trans.csv" ]; then
        evo_res "$BAG_PATH/evo/${agent}"/*/ape_trans_part.zip --save_table "$BAG_PATH/evo/${agent}/metrics_ape_trans.csv"
    else
        printWarning "Skipping APE trans metrics for ${agent} (already exists)"
    fi
    if [ ! -f "$BAG_PATH/evo/${agent}/metrics_ape_rot.csv" ]; then
        evo_res "$BAG_PATH/evo/${agent}"/*/ape_angle_deg.zip  --save_table "$BAG_PATH/evo/${agent}/metrics_ape_rot.csv"
    else
        printWarning "Skipping APE rot metrics for ${agent} (already exists)"
    fi
    if [ ! -f "$BAG_PATH/evo/${agent}/metrics_rpe_trans.csv" ]; then
        evo_res "$BAG_PATH/evo/${agent}"/*/rpe_trans_part.zip --save_table "$BAG_PATH/evo/${agent}/metrics_rpe_trans.csv"
    else
        printWarning "Skipping RPE trans metrics for ${agent} (already exists)"
    fi
    if [ ! -f "$BAG_PATH/evo/${agent}/metrics_rpe_rot.csv" ]; then
        evo_res "$BAG_PATH/evo/${agent}"/*/rpe_angle_deg.zip  --save_table "$BAG_PATH/evo/${agent}/metrics_rpe_rot.csv"
    else
        printWarning "Skipping RPE rot metrics for ${agent} (already exists)"
    fi
done
