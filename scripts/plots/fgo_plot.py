#!/usr/bin/env python3
# Created by Nelson Durrant (w Gemini 3 Pro), Jan 2026

import argparse
import math
import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores

"""
FGO Plotting Tool

Analyzes and compares localization data (FGO, EKF, UKF, Truth) from a rosbag file.
Generates plots for 3D trajectory, pose error, and state comparisons.

Usage:
    python3 fgo_plot.py <bag_file> [--name <auv_namespace>]
"""


COLOR_MAP = {
    "FGO": "tab:green",
    "FGO (TM)": "tab:blue",
    "Truth": "k",
    "EKF": "tab:red",
    "UKF": "tab:orange",
}


def quaternion_to_rpy(orientation):
    """Converts a ROS Quaternion message to roll, pitch, yaw."""
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def get_odometry_pose_data(reader, connections, typestore):
    """Extracts 6DOF pose data (x, y, z, roll, pitch, yaw, t) from Odometry messages."""
    data = {"x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": [], "t": []}
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        data["x"].append(msg.pose.pose.position.x)
        data["y"].append(msg.pose.pose.position.y)
        data["z"].append(msg.pose.pose.position.z)

        roll, pitch, yaw = quaternion_to_rpy(msg.pose.pose.orientation)
        data["roll"].append(roll)
        data["pitch"].append(pitch)
        data["yaw"].append(yaw)

        data["t"].append(timestamp)
    return data


def get_velocity_data(reader, connections, typestore):
    """Extracts velocity data from geometry_msgs/TwistWithCovarianceStamped."""
    data = {"vx": [], "vy": [], "vz": [], "t": []}
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        data["vx"].append(msg.twist.twist.linear.x)
        data["vy"].append(msg.twist.twist.linear.y)
        data["vz"].append(msg.twist.twist.linear.z)
        data["t"].append(timestamp)
    return data


def get_bias_data(reader, connections, typestore):
    """
    Extracts IMU bias data from geometry_msgs/TwistWithCovarianceStamped.
    Mapping based on C++ code:
    Linear -> Accelerometer Bias
    Angular -> Gyroscope Bias
    """
    data = {
        "ax": [],
        "ay": [],
        "az": [],  # Accelerometer
        "gx": [],
        "gy": [],
        "gz": [],  # Gyroscope
        "t": [],
    }
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        # Linear part is Accelerometer Bias
        data["ax"].append(msg.twist.twist.linear.x)
        data["ay"].append(msg.twist.twist.linear.y)
        data["az"].append(msg.twist.twist.linear.z)

        # Angular part is Gyroscope Bias
        data["gx"].append(msg.twist.twist.angular.x)
        data["gy"].append(msg.twist.twist.angular.y)
        data["gz"].append(msg.twist.twist.angular.z)

        data["t"].append(timestamp)
    return data


def calculate_component_errors(truth_data, test_data):
    """Calculates error over time for all 6 components using interpolation."""
    truth_t = np.array(truth_data["t"])
    test_t = np.array(test_data["t"])

    errors = {}

    # Interpolate all components
    for key in ["x", "y", "z", "roll", "pitch", "yaw"]:
        truth_interp = np.interp(test_t, truth_t, np.array(truth_data[key]))
        test_values = np.array(test_data[key])

        if key in ["roll", "pitch", "yaw"]:
            # Handle angle wrapping for error calculation
            diff = truth_interp - test_values
            error = np.arctan2(np.sin(diff), np.cos(diff))
        else:
            error = truth_interp - test_values

        errors[key] = error

    # Return timestamps in seconds relative to start
    return (test_t - truth_t[0]) / 1e9, errors


def read_topics(reader, connections_map, config, extractor_func, typestore):
    """Generic helper to read a set of topics using a specific extractor function."""
    data_out = {}
    for algorithm, topic in config.items():
        try:
            if topic in connections_map:
                data_out[algorithm] = extractor_func(
                    reader, connections_map[topic], typestore
                )
            else:
                print(f"Warning: Topic '{topic}' for {algorithm} not found.")
        except Exception as e:
            print(f"Error reading {topic}: {e}")
    return data_out


def draw_dropouts(ax, dvl_data, start_time_sec=0):
    """Draws gray background for DVL dropouts (gaps > 1.0s)."""
    if "DVL" not in dvl_data or not dvl_data["DVL"]["t"]:
        return

    dvl_t = np.array(dvl_data["DVL"]["t"]) / 1e9 - start_time_sec
    diffs = np.diff(dvl_t)
    gap_indices = np.where(diffs > 1.0)[0] # Threshold for gap detection

    for idx in gap_indices:
        start_gap = dvl_t[idx]
        end_gap = dvl_t[idx + 1]
        ax.axvspan(start_gap, end_gap, color="lightgray", alpha=0.5, label="_nolegend_")


def plot_3d_trajectory(odom_data, dvl_data, color_map):
    """Plots the 3D trajectory of all algorithms compared to Truth."""
    if "Truth" not in odom_data or not odom_data["Truth"]["x"]:
        return

    truth_data = odom_data["Truth"]
    fig = plt.figure(figsize=(10, 8))
    fig.canvas.manager.set_window_title("3D Trajectory Estimate")
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        truth_data["x"],
        truth_data["y"],
        truth_data["z"],
        label="Truth",
        color=color_map["Truth"],
    )

    for algorithm, data in odom_data.items():
        if algorithm == "Truth" or not data["x"]:
            continue
        ax.plot(
            data["x"],
            data["y"],
            data["z"],
            label=algorithm,
            color=color_map.get(algorithm),
        )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    ax.grid(True)
    print("Generated Combined 3D Trajectory Plot")


def plot_pose_errors(odom_data, dvl_data, color_map):
    """Calculates and plots the pose error (L2 norm) for all algorithms."""
    if "Truth" not in odom_data or not odom_data["Truth"]["t"]:
        return

    truth_data = odom_data["Truth"]
    fig, axs = plt.subplots(2, 3, figsize=(15, 6), sharex=True)
    fig.canvas.manager.set_window_title("Pose Error")
    ax_flat = axs.flatten()

    components = ["x", "y", "z", "roll", "pitch", "yaw"]
    titles = ["X Error", "Y Error", "Z Error", "Roll Error", "Pitch Error", "Yaw Error"]
    units = ["m", "m", "m", "rad", "rad", "rad"]

    for algorithm, data in odom_data.items():
        if algorithm == "Truth" or not data["x"]:
            continue

        try:
            timestamps_sec, errors = calculate_component_errors(truth_data, data)
            plot_color = color_map.get(algorithm, "tab:gray")

            for i, comp in enumerate(components):
                l2_norm = np.sqrt(np.mean(errors[comp] ** 2))
                label = f"{algorithm} (L2: {l2_norm:.3f})"

                ax_flat[i].plot(
                    timestamps_sec,
                    np.abs(errors[comp]),
                    color=plot_color,
                    alpha=0.9,
                    label=label,
                )
        except Exception as e:
            print(f"Could not calculate errors for {algorithm}: {e}")

    for i, ax in enumerate(ax_flat):
        ax.set_ylabel(f"{titles[i]} ({units[i]})")
        ax.grid(True)
        ax.set_ylim(bottom=0)
        ax.legend(fontsize="small", loc="upper right")

    for ax in axs[1, :]:
        ax.set_xlabel("Time (s)")

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    print("Generated L2 Norm Pose Error Plot.")

    # Apply dropout shading to all subplots
    start_time_sec = truth_data["t"][0] / 1e9
    for ax in ax_flat:
        draw_dropouts(ax, dvl_data, start_time_sec)



def plot_state_comparison(odom_data, vel_data, bias_data, dvl_data, color_map, start_time):
    """Plots a comprehensive comparison of all available states."""
    if not (odom_data or vel_data or bias_data):
        return

    fig, axs = plt.subplots(5, 3, figsize=(15, 10), sharex=True)
    fig.canvas.manager.set_window_title("State Comparison")

    row_configs = [
        (odom_data, ["x", "y", "z"], "m", "tab:gray"),
        (odom_data, ["roll", "pitch", "yaw"], "rad", "tab:gray"),
        (vel_data, ["vx", "vy", "vz"], "m/s", "tab:cyan"),
        (bias_data, ["ax", "ay", "az"], "m/s²", "tab:blue"),
        (bias_data, ["gx", "gy", "gz"], "rad/s", "tab:blue"),
    ]

    legend_handles = {}

    for row_idx, (source, components, unit, default_color) in enumerate(row_configs):
        if not source:
            continue

        for algorithm, data in source.items():
            if not data.get(components[0]):
                continue  # Skip empty

            # Auto-detect start time if not provided
            if start_time == 0 and data.get("t"):
                start_time = data["t"][0]

            t_sec = (np.array(data["t"]) - start_time) / 1e9
            color = color_map.get(algorithm, default_color)

            for col_idx, comp in enumerate(components):
                ax = axs[row_idx, col_idx]
                (line,) = ax.plot(t_sec, data[comp], color=color, alpha=0.8)

                label_str = (
                    comp.capitalize()
                    if comp in ["roll", "pitch", "yaw"]
                    else comp.upper()
                )
                ax.set_ylabel(f"{label_str} ({unit})")

                if algorithm not in legend_handles:
                    legend_handles[algorithm] = line

    # Explicitly plot DVL data on velocity row (row 2)
    if dvl_data and "DVL" in dvl_data:
        data = dvl_data["DVL"]
        if data.get("vx") and data.get("t"):
            t_sec = (np.array(data["t"]) - start_time) / 1e9
            dvl_comps = ["vx", "vy", "vz"]
            for col_idx, comp in enumerate(dvl_comps):
                ax = axs[2, col_idx]
                ax.plot(t_sec, data[comp], color="k", alpha=0.6, linewidth=1.0, label="_nolegend_")

    for r in range(5):
        for c in range(3):
            axs[r, c].grid(True)
            if r == 4:
                axs[r, c].set_xlabel("Time (s)")
            
            # Apply dropout shading
            start_time_sec = 0
            # Reuse logic from plot_state_comparison's start_time calculation is hard here because it's implicit
            # Let's assume start_time passed to function is correct
            if start_time != 0:
                 draw_dropouts(axs[r, c], dvl_data, start_time / 1e9)


    fig.legend(
        legend_handles.values(),
        legend_handles.keys(),
        loc="upper center",
        bbox_to_anchor=(0.5, 0.98),
        ncol=len(legend_handles),
        frameon=False,
    )
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    print("Generated System State Comparison Plot.")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze localization data from a rosbag file."
    )
    parser.add_argument(
        "bag_file", type=str, help="Path to the rosbag file (directory)."
    )
    parser.add_argument(
        "--name",
        type=str,
        default="/auv0",
        help="Name of the AUV namespace (default: /auv0).",
    )
    args = parser.parse_args()

    auv = args.name
    if not auv.startswith("/"):
        auv = f"/{auv}"

    odom_topics = {
        "EKF": f"{auv}/odometry/global_ekf",
        "UKF": f"{auv}/odometry/global_ukf",
        "FGO (TM)": f"{auv}/tm/odometry/global",
        "FGO": f"{auv}/odometry/global",
        "Truth": f"{auv}/odometry/truth",
    }

    vel_topics = {
        "Truth": f"{auv}/VelocitySensor",
        "FGO (TM)": f"{auv}/tm/factor_graph_node/velocity",
        "FGO": f"{auv}/factor_graph_node/velocity",
    }

    dvl_topics = {
        "DVL": f"{auv}/dvl/data",
    }

    bias_topics = {
        "Truth": f"{auv}/IMUSensorBias",
        "FGO (TM)": f"{auv}/tm/factor_graph_node/imu_bias",
        "FGO": f"{auv}/factor_graph_node/imu_bias",
    }

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(args.bag_file) as reader:
        connections_by_topic = {}
        for c in reader.connections:
            connections_by_topic.setdefault(c.topic, []).append(c)

        print("Reading data...")
        odom_data = read_topics(
            reader, connections_by_topic, odom_topics, get_odometry_pose_data, typestore
        )
        vel_data = read_topics(
            reader, connections_by_topic, vel_topics, get_velocity_data, typestore
        )
        dvl_data = read_topics(
            reader, connections_by_topic, dvl_topics, get_velocity_data, typestore
        )
        bias_data = read_topics(
            reader, connections_by_topic, bias_topics, get_bias_data, typestore
        )

    # Determine start time for relative plotting
    start_time = 0
    for data in [odom_data, vel_data, bias_data]:
        if "Truth" in data and data["Truth"]["t"]:
            start_time = data["Truth"]["t"][0]
            break

    plot_3d_trajectory(odom_data, dvl_data, COLOR_MAP)
    plot_pose_errors(odom_data, dvl_data, COLOR_MAP)
    plot_state_comparison(odom_data, vel_data, bias_data, dvl_data, COLOR_MAP, start_time)

    print("Analysis complete. Displaying plots...")
    plt.show()


if __name__ == "__main__":
    main()
