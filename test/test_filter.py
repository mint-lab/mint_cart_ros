#!/usr/bin/env python3
import csv
from collections import deque
import numpy as np
import sys
import matplotlib.pyplot as plt

def read_pressure_data(file_path: str) -> list:
    """Read a barometric data file and return the pressure data as a list"""
    pressure_list = []
    try:
        with open(file_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pressure_list.append(float(row['Pressure']))
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' does not exist!")
    except KeyError:
        print(f"Error: There is no 'Pressure' column in the file!")
    except ValueError:
        print(f"Error: There is non-numeric value in the 'Pressure' column!")
    except Exception as e:
        print(f"An error occurred: {e}")

    return pressure_list

def moving_average_filter(data_list: deque, measurement: float, window_size: int) -> tuple[float, deque]:
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

def find_flat_section(data_list, threshold=0.0002):
    """Find the flat section in the data"""
    gradient = np.gradient(data_list) # np.gradient vs np.diff ??
    flat_section = np.where(np.abs(gradient) < threshold)[0] # np.where returns tuple
    return flat_section

def calculate_cluster(flat_section, data_list, tolerance=100):
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

def plot_data(measurement, filtered_data, flat_section, cluster_list, cluster_average_list):
    """Plot the measurement data, the filtered data and the flat section"""
    plt.figure(figsize=(20, 10))
    plt.plot(measurement, 'r*', label='Measured')
    plt.plot(filtered_data, 'b-', label='Filtered')
    plt.plot(flat_section, [filtered_data[i] for i in flat_section],'g+', label='Flat')

    # Put the average of the cluster into the text
    for cluster, average in zip(cluster_list, cluster_average_list):
        cluster_mid_point = cluster[len(cluster) // 2]
        plt.text(cluster_mid_point, average, f'{average}', fontsize=10, ha='center', va='bottom')

    plt.title('Measure Pressure v.s. Moving Average Filter Values')
    plt.xlabel('Index')
    plt.ylabel('Pressure [hpa]')
    plt.legend()
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_filter.py <path_to_csv_file>")
        sys.exit()

    file_path = sys.argv[1]
    pressure_data_list = read_pressure_data(file_path)
    print(len(pressure_data_list), pressure_data_list[0:5])

    filtered_data_list = []
    pressure_window = deque()
    window_size = 100
    print(f'window_size={window_size}')
    for measurement in pressure_data_list:
        average, pressure_window = moving_average_filter(pressure_window, measurement, window_size)
        filtered_data_list.append(average)

    print(len(filtered_data_list), filtered_data_list[0:5])

    # Find flat section
    flat_section = find_flat_section(filtered_data_list)
    print(flat_section, type(flat_section))

    # Find clusters and calculate average of each cluster
    cluster_list, cluster_average_list = calculate_cluster(flat_section, pressure_data_list)

    # Create the plot
    plot_data(pressure_data_list, filtered_data_list, flat_section, cluster_list, cluster_average_list)

if __name__ == "__main__":
    main()
