#!/usr/bin/env python3
# Created by Nelson Durrant (w Gemini 3 Pro), Jan 2026

import glob
import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from pathlib import Path

ALGORITHMS = ["FGO", "TM", "EKF", "UKF", "IEKF"] # , "DVL"]

COLORS = {
    "FGO": "#4C72B0",
    "TM": "#DD8452",
    "EKF": "#55A868",
    "UKF": "#C44E52",
    "IEKF": "#8172B2",
    # "DVL": "#FFC107",
    "Truth": "black",
}

NAME_MAPPING = {
    "global": "FGO",
    "global_tm": "TM",
    "global_ekf": "EKF",
    "global_ukf": "UKF",
    "global_iekf": "IEKF",
    # "dvl": "DVL",
}

METRICS_CONFIG = [
    ("metrics_ape_trans.csv", "APE Translation RMSE (m)", "ape_trans"),
    ("metrics_ape_rot.csv", "APE Rotation RMSE (deg)", "ape_rot"),
    ("metrics_rpe_trans.csv", "RPE Translation RMSE (m/m)", "rpe_trans"),
    ("metrics_rpe_rot.csv", "RPE Rotation RMSE (deg/m)", "rpe_rot"),
]

from evo.tools.settings import SETTINGS
from evo.tools.plot import apply_settings

SETTINGS.plot_figsize = [3.5, 3.0]
SETTINGS.plot_fontfamily = "serif"
SETTINGS.plot_seaborn_style = "whitegrid"
SETTINGS.plot_usetex = True

apply_settings(SETTINGS)
sns.set_context("paper")


def load_data(bags_dir):
    data_store = {cfg[0]: [] for cfg in METRICS_CONFIG}
    files = glob.glob(os.path.join(bags_dir, "**", "metrics_*.csv"), recursive=True)

    print(f"Found {len(files)} metric files.")

    for f in files:
        filename = os.path.basename(f)
        if filename not in data_store:
            continue

        try:
            df = pd.read_csv(f, index_col=0)
            for algo_key, row in df.iterrows():
                label = NAME_MAPPING.get(algo_key, algo_key)
                if label in ALGORITHMS:
                    data_store[filename].append(
                        {"Algorithm": label, "RMSE": row["rmse"]}
                    )
        except Exception as e:
            print(f"Error reading {f}: {e}")

    return {k: pd.DataFrame(v) for k, v in data_store.items() if v}


def generate_plots(data_map, output_dir):
    if not data_map:
        print("No data found to plot.")
        return

    for filename, label, suffix in METRICS_CONFIG:
        if filename not in data_map:
            print(f"No data for {filename}, skipping.")
            continue

        df = data_map[filename]

        unique_algos = df["Algorithm"].unique()
        present_algos = [algo for algo in ALGORITHMS if algo in unique_algos]
        missing_algos = [algo for algo in ALGORITHMS if algo not in unique_algos]

        if missing_algos:
            print(
                f"The following algorithms are missing from {filename}: {missing_algos}"
            )

        if not present_algos:
            print(f"No valid algorithms found in data for {filename}, skipping.")
            continue

        df["Algorithm"] = pd.Categorical(
            df["Algorithm"], categories=present_algos, ordered=True
        )
        df = df.sort_values("Algorithm")

        for plot_type in ["violin", "box"]:
            plt.figure(figsize=SETTINGS.plot_figsize)
            if plot_type == "violin":
                sns.violinplot(
                    x="Algorithm",
                    y="RMSE",
                    hue="Algorithm",
                    data=df,
                    inner="box",
                    palette=COLORS,
                )
            else:
                sns.boxplot(
                    x="Algorithm",
                    y="RMSE",
                    hue="Algorithm",
                    data=df,
                    palette=COLORS,
                )

            plt.title("")
            plt.ylabel(label)
            plt.xlabel("")

            save_path = output_dir / f"{plot_type}_{suffix}.png"
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"Saved {save_path}")
            plt.close()


def main():
    bags_dir = Path(__file__).parent.parent.parent / "bags"
    if not bags_dir.exists():
        print(f"Error: {bags_dir} does not exist.")
        return

    print("Loading metrics data...")
    data = load_data(bags_dir)

    print("Generating metrics plots...")
    generate_plots(data, bags_dir)
    print("Done.")


if __name__ == "__main__":
    main()
