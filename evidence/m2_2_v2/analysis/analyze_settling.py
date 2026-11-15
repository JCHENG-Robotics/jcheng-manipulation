#!/usr/bin/env python3

from pathlib import Path
import csv

import numpy as np
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_PATH = (
    Path.home()
    / "ros2_test_data/m2_2/trajectory_settling_test"
)

OUTPUT_DIR = (
    Path.home()
    / "ros2_test_data/m2_2/analysis_settling"
)

TOPIC = "/arm_controller/controller_state"
THRESHOLD = 0.001  # rad


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

        names = list(msg.joint_names)

        if joint_names is None:
            joint_names = names

        if (
            names != joint_names
            or len(msg.reference.positions) != len(joint_names)
            or len(msg.feedback.positions) != len(joint_names)
        ):
            continue

        timestamps.append(timestamp * 1e-9)
        references.append(list(msg.reference.positions))
        feedbacks.append(list(msg.feedback.positions))

    if len(timestamps) < 2:
        raise RuntimeError("Insufficient controller state data")

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
        raise RuntimeError("Non-finite trajectory data")

    # Identify the last significant reference-position change.
    change = np.max(np.abs(np.diff(ref, axis=0)), axis=1)

    moving_indices = np.where(change > 1e-7)[0]

    if len(moving_indices) == 0:
        raise RuntimeError("No trajectory movement detected")

    stop_idx = moving_indices[-1] + 1
    stop_time = t[stop_idx]

    post_t = t[stop_idx:] - stop_time
    post_ref = ref[stop_idx:]
    post_actual = actual[stop_idx:]
    post_error = post_ref - post_actual

    # Export signed errors for subsequent system identification.
    csv_path = OUTPUT_DIR / "settling_data.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s"] + [f"{name}_error_rad" for name in joint_names])
        for i in range(len(post_t)):
            writer.writerow([float(post_t[i])] + post_error[i].tolist())

    print("\n=== Settling Analysis ===")
    print(f"Stop time index: {stop_idx}")
    print(f"Post-stop duration: {post_t[-1]:.3f} s")
    print(f"Threshold: ±{THRESHOLD:.4f} rad")

    for j, name in enumerate(joint_names):
        e = np.abs(post_error[:, j])

        initial_error = e[0]
        final_error = e[-1]

        # First point after which all remaining samples
        # stay within the threshold.
        settling_time = None

        for i in range(len(e)):
            if np.all(e[i:] <= THRESHOLD):
                settling_time = post_t[i]
                break

        print(f"\n{name}")
        print(f"  Stop error:  {initial_error:.8f} rad")
        print(f"  Final error: {final_error:.8f} rad")

        if settling_time is None:
            print("  Settling time: NOT REACHED")
        else:
            print(f"  Settling time: {settling_time:.4f} s")

        for check_time in [1.0, 3.0, 5.0]:
            if post_t[-1] >= check_time:
                value = np.interp(check_time, post_t, e)
                print(
                    f"  Error at {check_time:.0f}s: "
                    f"{value:.8f} rad"
                )
            else:
                print(
                    f"  Error at {check_time:.0f}s: "
                    "NOT RECORDED"
                )

        plt.figure(figsize=(10, 5))

        plt.plot(post_t, e, label="Absolute tracking error")

        plt.axhline(
            THRESHOLD,
            linestyle="--",
            label="Settling threshold",
        )

        plt.xlabel("Time after reference stops (s)")
        plt.ylabel("Absolute position error (rad)")
        plt.title(f"{name} Post-Stop Error")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            OUTPUT_DIR / f"{name}_settling.png",
            dpi=160,
        )

        plt.close()

    print(f"\nCSV saved to: {csv_path}")
    print(f"Results saved to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
