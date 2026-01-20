#!/usr/bin/env python3
# Created by Nelson Durrant (w Gemini 3 Pro), Jan 2026

import glob
import os
from pathlib import Path

from evo.tools import file_interface
from evo.tools import plot
from evo.tools.settings import SETTINGS
import matplotlib.pyplot as plt
import seaborn as sns

ALGORITHMS = ["FGO", "TM", "EKF", "UKF", "IEKF"]

COLORS = {
    "FGO": "#4C72B0",
    "TM": "#DD8452",
    "EKF": "#55A868",
    "UKF": "#C44E52",
    "IEKF": "#8172B2",
    "Truth": "black",
}

NAME_MAPPING = {
    "fgo": "FGO",
    "tm": "TM",
    "ekf": "EKF",
    "ukf": "UKF",
    "iekf": "IEKF",
}

SETTINGS.plot_figsize = [3.5, 3.0]
SETTINGS.plot_fontfamily = "serif"
SETTINGS.plot_seaborn_style = "whitegrid"
SETTINGS.plot_usetex = True

plot.apply_settings(SETTINGS)
sns.set_context("paper")


def add_start_end_markers(
    ax,
    traj,
    start_symbol="o",
    start_color="black",
    end_symbol="x",
    end_color="black",
    alpha=1.0,
):
    if traj.num_poses == 0:
        return
    start = traj.positions_xyz[0]
    end = traj.positions_xyz[-1]

    # Assume XY mode
    x_idx, y_idx = 0, 1

    start_coords = [start[x_idx], start[y_idx]]
    end_coords = [end[x_idx], end[y_idx]]

    ax.scatter(
        *start_coords,
        marker=start_symbol,
        color=start_color,
        alpha=alpha,
    )
    ax.scatter(
        *end_coords,
        marker=end_symbol,
        color=end_color,
        alpha=alpha,
    )


def load_trajectories(evo_agent_dir):
    est_trajs = {}
    gt_traj = None

    zips = glob.glob(os.path.join(evo_agent_dir, "**", "*.zip"), recursive=True)

    print(f"Found {len(zips)} trajectory files.")

    for z in zips:
        if "ape_trans" not in z:
            continue

        parent = Path(z).parent.name

        algo_label = None
        for key, mapped_name in NAME_MAPPING.items():
            if key in parent:
                algo_label = mapped_name
                break

        if algo_label not in ALGORITHMS:
            continue

        try:
            res = file_interface.load_res_file(z, load_trajectories=True)

            ref_id = Path(res.info["ref_name"]).name
            est_id = Path(res.info["est_name"]).name

            if gt_traj is None:
                if ref_id in res.trajectories:
                    gt_traj = res.trajectories[ref_id]

            if est_id in res.trajectories:
                est_trajs[algo_label] = res.trajectories[est_id]

        except Exception as e:
            print(f"Error loading {z}: {e}")

    return est_trajs, gt_traj


def plot_auv(evo_agent_dir, output_dir, auv_name):
    est_trajs, gt_traj = load_trajectories(evo_agent_dir)

    present_algos = list(est_trajs.keys())
    missing_algos = [algo for algo in ALGORITHMS if algo not in present_algos]

    if missing_algos:
        print(f"The following algorithms are missing from {auv_name}: {missing_algos}")

    if gt_traj is None:
        print(f"No truth trajectory found for {auv_name}, skipping.")
        return

    fig = plt.figure(figsize=SETTINGS.plot_figsize)
    ax = plot.prepare_axis(fig, plot.PlotMode.xy)
    ax.set_xlabel("$x$ (m)")
    ax.set_ylabel("$y$ (m)")

    plot.traj(
        ax,
        plot.PlotMode.xy,
        gt_traj,
        style="--",
        color=COLORS["Truth"],
        label="Truth",
    )
    add_start_end_markers(
        ax,
        gt_traj,
        start_color=COLORS["Truth"],
        end_color=COLORS["Truth"],
    )

    for algo in ALGORITHMS:
        if algo in est_trajs:
            plot.traj(
                ax,
                plot.PlotMode.xy,
                est_trajs[algo],
                style="-",
                color=COLORS[algo],
                alpha=0.8,
                label=algo,
            )
            add_start_end_markers(
                ax,
                est_trajs[algo],
                start_color=COLORS[algo],
                end_color=COLORS[algo],
                alpha=0.8,
            )

    ax.set_title("")
    plt.legend(frameon=True)

    output_path = os.path.join(output_dir, f"{auv_name}_trajectories.png")
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Saved {output_path}")
    plt.close(fig)


def main():
    bags_root = Path(__file__).parent.parent.parent / "bags"
    if not bags_root.exists():
        print(f"Error: {bags_root} does not exist.")
        return

    print("Loading trajectory data and generating plots...")

    for bag_dir in bags_root.iterdir():
        if not bag_dir.is_dir():
            continue

        evo_dir = bag_dir / "evo"
        if not evo_dir.exists():
            continue

        for agent_dir in evo_dir.iterdir():
            if agent_dir.is_dir() and agent_dir.name.startswith("auv"):
                plot_auv(str(agent_dir), str(bag_dir), agent_dir.name)

    print("Done.")


if __name__ == "__main__":
    main()
