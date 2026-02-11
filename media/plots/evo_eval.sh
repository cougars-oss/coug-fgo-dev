#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Generates APE/RPE benchmarks for FGO evaluation
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

script_dir="$(dirname "$(readlink -f "$0")")"
source "$script_dir/../../scripts/utils/common.sh"

if [ -z "$1" ]; then
    print_error "Usage: $0 <bag_name> [evo_args...]"
    exit 1
fi

bag_name="$1"
bag_path="$script_dir/../../bags/$bag_name"
shift

if [ ! -d "$bag_path" ]; then
    print_error "Bag directory not found: $bag_path"
    exit 1
fi

metadata_file="$bag_path/metadata.yaml"
if [ ! -f "$metadata_file" ]; then
    print_error "Metadata file not found: $metadata_file"
    exit 1
fi

evo_config reset &>/dev/null
evo_config set save_traj_in_zip true &>/dev/null

AGENTS=("coug0sim" "coug1sim" "coug2sim" "blue0sim" "bluerov2")
SUFFIXES=("odometry/global" "odometry/global_tm" \
          "odometry/global_ekf" "odometry/global_ukf" \
          "odometry/global_iekf" "odometry/dvl")
LABELS=("fgo" "tm" "ekf" "ukf" "iekf" "dvl")
METRICS=("trans_part" "angle_deg")

for agent in "${AGENTS[@]}"; do
    truth="/${agent}/odometry/truth"

    if ! grep -q "name: $truth" "$metadata_file"; then
        print_warning "Agent $agent not found, skipping..."
        continue
    fi

    print_info "Processing $agent..."

    est_topics=()
    for s in "${SUFFIXES[@]}"; do est_topics+=("/${agent}/${s}"); done

    for i in "${!est_topics[@]}"; do
        topic="${est_topics[$i]}"
        label="${LABELS[$i]}"

        if ! grep -q "name: $topic" "$metadata_file"; then
            print_warning "Topic $topic not found, skipping..."
            continue
        fi

        print_info "Evaluating ${agent}/${label}..."

        for metric in "${METRICS[@]}"; do
            mkdir -p "$bag_path/evo/${agent}/${label}"

            # 1. APE (Global Accuracy)
            if [ ! -f "$bag_path/evo/${agent}/${label}/ape_${metric}.zip" ]; then
                evo_ape bag2 "$bag_path" "$truth" "$topic" -r "$metric" "$@" \
                    --save_results "$bag_path/evo/${agent}/${label}/ape_${metric}.zip"
            else
                print_warning "Skipping APE ${metric} for ${agent}/${label} (already exists)"
            fi

            # 2. RPE (Drift - Normalized per 1 meter)
            if [ ! -f "$bag_path/evo/${agent}/${label}/rpe_${metric}.zip" ]; then
                evo_rpe bag2 "$bag_path" "$truth" "$topic" -r "$metric" "$@" \
                    --delta 1 --delta_unit m --all_pairs \
                    --save_results "$bag_path/evo/${agent}/${label}/rpe_${metric}.zip"
            else
                print_warning "Skipping RPE ${metric} for ${agent}/${label} (already exists)"
            fi
        done
    done

    print_info "Exporting ${agent} benchmark..."
    if [ ! -f "$bag_path/evo/${agent}/benchmark_ape_trans.csv" ]; then
        evo_res "$bag_path/evo/${agent}"/*/ape_trans_part.zip --save_table "$bag_path/evo/${agent}/benchmark_ape_trans.csv"
    else
        print_warning "Skipping APE trans benchmark for ${agent} (already exists)"
    fi
    if [ ! -f "$bag_path/evo/${agent}/benchmark_ape_rot.csv" ]; then
        evo_res "$bag_path/evo/${agent}"/*/ape_angle_deg.zip  --save_table "$bag_path/evo/${agent}/benchmark_ape_rot.csv"
    else
        print_warning "Skipping APE rot benchmark for ${agent} (already exists)"
    fi
    if [ ! -f "$bag_path/evo/${agent}/benchmark_rpe_trans.csv" ]; then
        evo_res "$bag_path/evo/${agent}"/*/rpe_trans_part.zip --save_table "$bag_path/evo/${agent}/benchmark_rpe_trans.csv"
    else
        print_warning "Skipping RPE trans benchmark for ${agent} (already exists)"
    fi
    if [ ! -f "$bag_path/evo/${agent}/benchmark_rpe_rot.csv" ]; then
        evo_res "$bag_path/evo/${agent}"/*/rpe_angle_deg.zip  --save_table "$bag_path/evo/${agent}/benchmark_rpe_rot.csv"
    else
        print_warning "Skipping RPE rot benchmark for ${agent} (already exists)"
    fi
done
