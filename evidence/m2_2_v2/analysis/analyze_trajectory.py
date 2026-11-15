#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_PATH = Path.home() / "ros2_test_data/m2_2/trajectory_test_01"
OUTPUT_DIR = Path.home() / "ros2_test_data/m2_2/analysis"
TOPIC = "/arm_controller/controller_state"


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(BAG_PATH),
            storage_id="mcap",
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    msg_type = get_message(topic_types[TOPIC])

    timestamps = []
    references = []
    feedbacks = []

    joint_names = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != TOPIC:
            continue

        msg = deserialize_message(data, msg_type)

        if joint_names is None:
            joint_names = list(msg.joint_names)

        if (
            list(msg.joint_names) != joint_names
            or len(msg.reference.positions) != len(joint_names)
            or len(msg.feedback.positions) != len(joint_names)
        ):
            continue

        timestamps.append(timestamp * 1e-9)
        references.append(list(msg.reference.positions))
        feedbacks.append(list(msg.feedback.positions))

    if len(timestamps) < 2:
        raise RuntimeError("Not enough valid controller state messages")

    t = np.asarray(timestamps)
    ref = np.asarray(references)
    actual = np.asarray(feedbacks)

    order = np.argsort(t)
    t = t[order]
    ref = ref[order]
    actual = actual[order]

    if not (
        np.isfinite(t).all()
        and np.isfinite(ref).all()
        and np.isfinite(actual).all()
    ):
        raise RuntimeError("Non-finite values detected")

    # Detect changes in reference position.
    reference_change = np.max(np.abs(np.diff(ref, axis=0)), axis=1)

    moving_indices = np.where(reference_change > 1e-7)[0]

    if len(moving_indices) == 0:
        raise RuntimeError(
            "No reference trajectory movement detected. "
            "Check whether recording covered goal execution."
        )

    # Include the samples immediately surrounding detected movement.
    start_idx = max(0, moving_indices[0])
    end_idx = min(len(t), moving_indices[-1] + 2)

    t_segment = t[start_idx:end_idx] - t[start_idx]
    ref_segment = ref[start_idx:end_idx]
    actual_segment = actual[start_idx:end_idx]

    error = ref_segment - actual_segment

    print("\n=== Trajectory Tracking Analysis ===")
    print(f"Samples: {len(t_segment)}")
    print(f"Detected interval: {t_segment[-1]:.4f} s")

    for j, name in enumerate(joint_names):
        e = error[:, j]

        mae = np.mean(np.abs(e))
        rmse = np.sqrt(np.mean(e**2))
        max_error = np.max(np.abs(e))
        final_error = abs(e[-1])

        print(f"\n{name}")
        print(f"  MAE:         {mae:.8f} rad")
        print(f"  RMSE:        {rmse:.8f} rad")
        print(f"  Max Error:   {max_error:.8f} rad")
        print(f"  Final Error: {final_error:.8f} rad")

    csv_path = OUTPUT_DIR / "trajectory_tracking.csv"

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)

        header = ["time_s"]

        for name in joint_names:
            header.extend([
                f"{name}_reference_rad",
                f"{name}_actual_rad",
                f"{name}_error_rad",
            ])

        writer.writerow(header)

        for i in range(len(t_segment)):
            row = [t_segment[i]]

            for j in range(len(joint_names)):
                row.extend([
                    ref_segment[i, j],
                    actual_segment[i, j],
                    error[i, j],
                ])

            writer.writerow(row)

    for j, name in enumerate(joint_names):
        plt.figure(figsize=(10, 5))

        plt.plot(
            t_segment,
            ref_segment[:, j],
            label="Reference",
        )

        plt.plot(
            t_segment,
            actual_segment[:, j],
            label="Actual",
            linestyle="--",
        )

        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.title(f"{name} Trajectory Tracking")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        plt.savefig(
            OUTPUT_DIR / f"{name}_tracking.png",
            dpi=160,
        )

        plt.close()

    print(f"\nResults saved to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
