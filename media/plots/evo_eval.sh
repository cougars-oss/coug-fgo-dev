#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Generates APE/RPE benchmark metrics for FGO evaluation
#
# Usage:
#   ./evo_eval.sh <bag_name> [-n <N>] [-a] [-o] [-2]
#
# Arguments:
#   <bag_name>: Evaluate ../../bags/<bag_name> (required)
#   -a: Align trajectories using Umeyama's method (best fit)
#   -o: Align trajectories using the first pose (origin)
#   -2: Project trajectories to the 2D plane (xy)
#   -n: Number of poses to use for alignment (default: all)

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/../../scripts/common.sh"
source "$SCRIPT_DIR/../../.venv/bin/activate"

if [ -z "$1" ]; then
    printError "Usage: $0 <bag_name> [-n <N>] [-a] [-o] [-2]"
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

ALIGN=""
ALIGN_ORIGIN=""
TWO_D=""
N_TO_ALIGN=""

while getopts ":n:ao2" opt; do
    case $opt in
        a)
            ALIGN="--align"
            ;;
        o)
            ALIGN_ORIGIN="--align_origin"
            ;;
        2)
            TWO_D="--project_to_plane xy"
            ;;
        n)
            N_TO_ALIGN="--n_to_align $OPTARG"
            ALIGN="--align"
            ;;
        \?)
            printError "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
    esac
done

EVO_ARGS="$ALIGN $ALIGN_ORIGIN $TWO_D $N_TO_ALIGN"

# evo_config reset
evo_config set save_traj_in_zip true

AGENTS=("auv0" "auv1" "auv2" "bluerov2")
SUFFIXES=("odometry/global" "odometry/global_tm" "odometry/global_ekf" "odometry/global_ukf" "odometry/global_iekf" "odometry/dvl")
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
                evo_ape bag2 "$BAG_PATH" "$TRUTH" "$TOPIC" -r "$metric" $EVO_ARGS \
                    --save_results "$BAG_PATH/evo/${agent}/${LABEL}/ape_${metric}.zip"
            else
                printWarning "Skipping APE ${metric} for ${agent}/${LABEL} (already exists)"
            fi

            # 2. RPE (Drift - Normalized per 1 meter)
            if [ ! -f "$BAG_PATH/evo/${agent}/${LABEL}/rpe_${metric}.zip" ]; then
                evo_rpe bag2 "$BAG_PATH" "$TRUTH" "$TOPIC" -r "$metric" $EVO_ARGS \
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
