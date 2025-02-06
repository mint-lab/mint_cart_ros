#!/usr/bin/env python3
import argparse
from pathlib import Path
from collections import deque
import csv

import numpy as np
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# Constants
T_grad = 0.0065  # Temperature lapse rate [K/m]
R = 287.058  # Specific gas constant for dry air [J/(kg*K)]
g = 9.80665  # Gravitational acceleration [m/s^2]
GT_MIRAE = [
    0.0,
    2.178,
    4.299,
    6.456,
    8.542,
    10.777,
    12.785,
    14.993,
    17.012,
    19.213,
    21.246,
]
GT_MUGUNG = [0.0, 2.796, 5.284, 8.080, 10.568, 13.364, 15.852]


def load_barometric_data(bag_file: str, topic_name: str) -> list:
    """Read `bag_file` and extract barometric data named as `topic_name`"""
    bag_file = Path(bag_file)
    if not topic_name.startswith("/"):
        topic_name = "/" + topic_name

    barometric_data = []
    try:
        with AnyReader(
            [bag_file], default_typestore=get_typestore(Stores.ROS2_HUMBLE)
        ) as reader:
            connections = [x for x in reader.connections if x.topic == topic_name]
            for connection, timestamp, raw_data in reader.messages(
                connections=connections
            ):
                msg = reader.deserialize(raw_data, connection.msgtype)
                barometric_data.append((timestamp, msg.fluid_pressure, msg.variance))
    except Exception as e:
        print(f"Error reading rosbag file: {e}")
        print("Attempting to read from CSV file...")

        # Attempt to read from a CSV file with the same name as the bag file
        csv_file = bag_file.with_suffix(".csv")
        if csv_file.exists():
            with open(csv_file, "r") as f:
                csv_reader = csv.reader(f)
                next(csv_reader)  # Skip header row if it exists
                for row in csv_reader:
                    timestamp, fluid_pressure, variance = map(float, row)
                    barometric_data.append((timestamp, fluid_pressure, variance))
        else:
            print(f"CSV file {csv_file} not found. Unable to load data.")

    return barometric_data


def moving_average_filter(
    data_list: deque, measurement: float, window_size: int
) -> tuple[float, deque]:
    """Apply a moving average filter to the pressure data"""
    # n = len(data_list)
    # for i in range(n - 1):
    #     data_list[i] = data_list[i + 1]
    # data_list[n - 1] = measurement
    if len(data_list) == window_size:
        data_list.popleft()
    data_list.append(measurement)
    average = np.mean(data_list)
    return average, data_list


def find_flat_section(data_list, threshold=0.0002) -> tuple:
    """Find the flat section in the data"""
    gradient = np.gradient(data_list)  # np.gradient vs np.diff ??
    flat_section = np.where(np.abs(gradient) < threshold)[0]  # np.where returns tuple
    return flat_section


def calculate_cluster(flat_section, data_list, tolerance=120) -> tuple[list, list]:
    """Find clusters and calculate the average of each cluster"""
    cluster_list = []
    current_cluster = []

    for i in range(len(flat_section)):
        if i == 0:
            current_cluster.append(flat_section[i])
        else:
            if abs(flat_section[i] - current_cluster[-1]) <= tolerance:
                current_cluster.append(flat_section[i])
            else:
                cluster_list.append(current_cluster)
                current_cluster = [flat_section[i]]

    # Prevent missing last cluster
    if current_cluster:
        cluster_list.append(current_cluster)

    # Get average in clusters
    cluster_average_list = []
    for cluster in cluster_list:
        value_list = [data_list[i] for i in cluster]
        average = np.mean(value_list)
        cluster_average_list.append(average)

    return cluster_list, cluster_average_list


def calculate_relative_altitude(T_0, P_0, P_list):
    """
    @brief Calculate the relative altitude from barometric pressure
    @param T_0 The absolute temperature measured at the initial point [K]
    @param P_0 The pressure measured at the initial point [Pa]
    @param P_list A list of pressures measured at the sensor's position by changing the position [Pa]
    return A list of relative altitude [m]
    """
    # Define the delta_h function
    delta_h = lambda P: (T_0 / T_grad) * (1 - (P / P_0) ** (T_grad * R / g))

    # return [delta_h(p) for p in P_list]
    return [delta_h(p) for p in P_list]


def prepare_plotting_data(
    ros_bag_file: str,
    ros_topic_name: str,
    T0: float,
    window_size: int = 100,
):
    upward_pressure_list = []
    upward_altitude_list = []
    downward_pressure_list = []
    downward_altitude_list = []

    # Load barometric data from a ROS bag file
    barometric_data_list = load_barometric_data(ros_bag_file, ros_topic_name)

    # Get the data of flat section
    window_size = 100
    filtered_data_list = []
    pressure_window = deque()
    pressure_data_list = []
    for timestamp, fluid_pressure, variance in barometric_data_list:
        average, pressure_window = moving_average_filter(
            pressure_window, fluid_pressure, window_size
        )
        pressure_data_list.append(fluid_pressure)
        filtered_data_list.append(average)

    flat_index_list = find_flat_section(filtered_data_list)

    cluster_list, cluster_average_list = calculate_cluster(
        flat_index_list, pressure_data_list
    )

    P0 = cluster_average_list[0]
    relative_altitude_list = calculate_relative_altitude(
        T0 + 273.15, P0, cluster_average_list
    )

    mid_point = len(cluster_average_list) // 2
    upward_pressure_list = cluster_average_list[: mid_point + 1]
    upward_pressure_list = [up * 100 for up in upward_pressure_list]
    downward_pressure_list = cluster_average_list[mid_point:]
    downward_pressure_list = [dp * 100 for dp in downward_pressure_list]
    upward_altitude_list = relative_altitude_list[: mid_point + 1]
    downward_altitude_list = relative_altitude_list[mid_point:]

    return (
        upward_pressure_list,
        upward_altitude_list,
        downward_pressure_list,
        downward_altitude_list,
    )


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="A barometer data figure for paper")
    parser.add_argument(
        "day1_ros_bag_file", type=str, help="ROS bag file path for day1"
    )
    parser.add_argument("day1_T0", type=float, help="Initial temperature for day1")
    parser.add_argument(
        "day2_ros_bag_file", type=str, help="ROS bag file path for day2"
    )
    parser.add_argument("day2_T0", type=float, help="Initial temperature for day2")
    parser.add_argument(
        "day3_ros_bag_file", type=str, help="ROS bag file path for day3"
    )
    parser.add_argument("day3_T0", type=float, help="Initial temperature for day3")
    parser.add_argument(
        "--ros_topic_name",
        "-r",
        type=str,
        default="/zed2/zed_node/atm_press",
        help="Barometer pressure topic name",
    )
    args = parser.parse_args()

    day1_T0 = args.day1_T0
    (
        upward_pressure_list,
        upward_altitude_list,
        downward_pressure_list,
        downward_altitude_list,
    ) = prepare_plotting_data(
        args.day1_ros_bag_file,
        args.ros_topic_name,
        day1_T0,
    )
    # P = lambda delta_h: upward_pressure_list[0] * (1 - (delta_h * T_grad / day1_T0))
    # gt_day1 = [delta_h(p) for p in upward_pressure_list]

    day2_T0 = args.day2_T0
    (
        upward_pressure_list2,
        upward_altitude_list2,
        downward_pressure_list2,
        downward_altitude_list2,
    ) = prepare_plotting_data(
        args.day2_ros_bag_file,
        args.ros_topic_name,
        day2_T0,
    )

    day3_T0 = args.day3_T0
    (
        upward_pressure_list3,
        upward_altitude_list3,
        downward_pressure_list3,
        downward_altitude_list3,
    ) = prepare_plotting_data(
        args.day3_ros_bag_file,
        args.ros_topic_name,
        day3_T0,
    )

    # Covert the pressure data to hPa if the pressure unit is Pa
    if np.mean(upward_pressure_list) > 900:
        upward_pressure_list = [up / 100 for up in upward_pressure_list]
        downward_pressure_list = [dp / 100 for dp in downward_pressure_list]
    if np.mean(upward_pressure_list2) > 900:
        upward_pressure_list2 = [up / 100 for up in upward_pressure_list2]
        downward_pressure_list2 = [dp / 100 for dp in downward_pressure_list2]
    if np.mean(upward_pressure_list3) > 900:
        upward_pressure_list3 = [up / 100 for up in upward_pressure_list3]
        downward_pressure_list3 = [dp / 100 for dp in downward_pressure_list3]

    label_fontsize = 18
    legend_fontsize = 16
    ticks_fontsize = 16

    # Plot the pressure data(y-axis) with the true relative height(x-axis)
    plt.figure(figsize=(10, 7.5))
    plt.plot(
        GT_MIRAE,
        upward_pressure_list,
        "o",
        color="blue",
        linestyle="-.",
        label=f"Day1 upward pressure ({day1_T0:.1f} $\degree C$)",
    )
    plt.plot(
        GT_MIRAE,
        downward_pressure_list[::-1],
        "x",
        color="blue",
        linestyle=":",
        label=f"Day1 downward pressure ({day1_T0:.1f} $\degree C$)",
    )
    plt.plot(
        GT_MIRAE,
        upward_pressure_list2,
        "o",
        color="red",
        linestyle="-.",
        label=f"Day2 upward pressure ({day2_T0:.1f} $\degree C$)",
    )
    plt.plot(
        GT_MIRAE,
        downward_pressure_list2[::-1],
        "x",
        color="red",
        linestyle=":",
        label=f"Day2 downward pressure ({day2_T0:.1f} $\degree C$)",
    )
    plt.plot(
        GT_MIRAE,
        upward_pressure_list3,
        "o",
        color="green",
        linestyle="-.",
        label=f"Day3 upward pressure ({day3_T0:.1f} $\degree C$)",
    )
    plt.plot(
        GT_MIRAE,
        downward_pressure_list3[::-1],
        "x",
        color="green",
        linestyle=":",
        label=f"Day3 downward pressure ({day3_T0:.1f} $\degree C$)",
    )
    # plt.title("Changes in barometric data over time")
    plt.xlabel(r"$True\;height\;[m]$", fontsize=label_fontsize)
    plt.ylabel(r"$Air\;pressure\;[hPa]$", fontsize=label_fontsize)
    plt.xticks(fontsize=ticks_fontsize)
    plt.yticks(fontsize=ticks_fontsize)
    plt.grid(alpha=0.5)
    plt.tight_layout()
    plt.legend(loc="best", fontsize=legend_fontsize)

    # Plot the estimated relative height(y-axis) with the true relative height(x-axis)
    plt.figure(figsize=(7.5, 7.5))
    plt.plot(
        GT_MIRAE,
        upward_altitude_list,
        "o",
        color="blue",
        linestyle="-.",
        label=f"Day1 upward height ({day1_T0:.1f} $\degree C$)",
    )
    plt.plot(
        GT_MIRAE,
        downward_altitude_list[::-1],
        "x",
        color="blue",
        linestyle=":",
        label=f"Day1 downward height",
    )
    plt.plot(
        GT_MIRAE,
        upward_altitude_list2,
        "o",
        color="red",
        linestyle="-.",
        label=f"Day2 upward height ({day2_T0:.1f} $\degree C$)",
    )
    plt.plot(
        GT_MIRAE,
        downward_altitude_list2[::-1],
        "x",
        color="red",
        linestyle=":",
        label=f"Day2 downward height",
    )
    plt.plot(
        GT_MIRAE,
        upward_altitude_list3,
        "o",
        color="green",
        linestyle="-.",
        label=f"Day3 upward height ({day3_T0:.1f} $\degree C$)",
    )
    plt.plot(
        GT_MIRAE,
        downward_altitude_list3[::-1],
        "x",
        color="green",
        linestyle=":",
        label=f"Day3 downward height",
    )
    lower_bound = min(
        upward_altitude_list
        + upward_altitude_list2
        + upward_altitude_list3
        + downward_altitude_list
        + downward_altitude_list2
        + downward_altitude_list3
    )
    upper_bound = max(
        upward_altitude_list
        + upward_altitude_list2
        + upward_altitude_list3
        + downward_altitude_list
        + downward_altitude_list2
        + downward_altitude_list3
    )
    print(lower_bound, upper_bound)
    plt.plot(
        [GT_MIRAE[0], GT_MIRAE[-1]],
        [GT_MIRAE[0], GT_MIRAE[-1]],
        "o",
        color="black",
        linestyle="-",
        label="Ground truth",
    )
    plt.xlabel(r"$True\;height\;[m]$", fontsize=label_fontsize)
    plt.ylabel(r"$Relative\;height\;[m]$", fontsize=label_fontsize)
    plt.xticks(fontsize=ticks_fontsize)
    plt.yticks(fontsize=ticks_fontsize)
    plt.xlim(lower_bound - 0.5, upper_bound + 0.5)
    plt.ylim(lower_bound - 0.5, upper_bound + 0.5)
    plt.grid(alpha=0.5)
    plt.legend(fontsize=legend_fontsize)
    plt.tight_layout()
    # set the aspect ratio of the plot to be equal
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()
