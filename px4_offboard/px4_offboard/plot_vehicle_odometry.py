#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

__author__ = "Erik Fougner Arnesen"
__contact__ = "Erik.f.Arnesen@gmail.com"

# ------------- User settings -------------
# Path to your bag. You can use either the folder or the .db3 file.
BAG_URI = "/home/arnesen/ros2_px4_offboard_ws/test_hover_01"           # folder name
# BAG_URI = "test_hover_01_0.db3"   # or directly the db3 file

ODOM_TOPIC = "/fmu/out/vehicle_odometry"
# ----------------------------------------


def read_odometry_from_bag(bag_uri, odom_topic):
    """Read timestamps and positions from /fmu/out/vehicle_odometry."""
    # Set up rosbag2 reader
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_uri,
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Map topic name -> type string
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if odom_topic not in type_map:
        raise RuntimeError(f"Topic '{odom_topic}' not found in bag. "
                           f"Available topics: {list(type_map.keys())}")

    # Resolve message class for this topic
    msg_type_str = type_map[odom_topic]
    msg_type = get_message(msg_type_str)

    times_ns = []
    xs, ys, zs = [], [], []

    first_t = None

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic != odom_topic:
            continue

        msg = deserialize_message(data, msg_type)

        # Time: use bag timestamp (nanoseconds)
        if first_t is None:
            first_t = t
        times_ns.append(t - first_t)

        # Position fields: PX4 VehicleOdometry typically uses x, y, z
        # If your msg uses `position[3]` instead, adapt here.
        if hasattr(msg, "x") and hasattr(msg, "y") and hasattr(msg, "z"):
            xs.append(msg.x)
            ys.append(msg.y)
            zs.append(msg.z)
        elif hasattr(msg, "position"):
            # In case the field is an array position[3]
            xs.append(msg.position[0])
            ys.append(msg.position[1])
            zs.append(msg.position[2])
        else:
            raise RuntimeError("Could not find position fields (x,y,z or position[3]) "
                               "in VehicleOdometry message.")

    if not times_ns:
        raise RuntimeError(f"No messages read from topic '{odom_topic}'")

    # Convert to numpy arrays
    t_sec = np.array(times_ns, dtype=np.float64) * 1e-9
    x_arr = np.array(xs, dtype=np.float32)
    y_arr = np.array(ys, dtype=np.float32)
    z_arr = np.array(zs, dtype=np.float32)

    return t_sec, x_arr, y_arr, z_arr


def main():
    # If user gave the folder, make sure it exists
    if not os.path.exists(BAG_URI):
        raise FileNotFoundError(
            f"Bag URI '{BAG_URI}' not found. "
            f"Run this script from the directory that contains '{BAG_URI}'."
        )

    print(f"Reading odometry from bag: {BAG_URI}")
    t_sec, x, y, z = read_odometry_from_bag(BAG_URI, ODOM_TOPIC)

    # ------------- Plot -------------#
    plt.figure()
    plt.plot(t_sec, x, label="x [m]")
    plt.plot(t_sec, y, label="y [m]")
    plt.plot(t_sec, z, label="z [m]")

    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.title("VehicleOdometry position vs time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
