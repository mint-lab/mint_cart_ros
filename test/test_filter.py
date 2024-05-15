#!/usr/bin/env python3
import csv
from collections import deque
import numpy as np
import sys
import matplotlib.pyplot as plt

def read_pressure_data(file_path: str) -> list:
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

def moving_average_filter(data_list: deque, measurement):
    # n = len(data_list)
    # for i in range(n - 1):
    #     data_list[i] = data_list[i + 1]
    # data_list[n - 1] = measurement
    data_list.popleft()
    data_list.append(measurement)
    average = np.mean(data_list)
    return average, data_list

def plot_data(measurement, filtered_data):
    plt.figure(figsize=(10, 5))
    plt.plot(measurement, 'r*', label='Measured')
    plt.plot(filtered_data, 'b-', label='Filtered')
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
        if len(pressure_window) < window_size:
            average, pressure_window = measurement, deque([measurement] * window_size, maxlen=window_size)
        else:
            average, pressure_window = moving_average_filter(pressure_window, measurement)
        filtered_data_list.append(average)

    print(len(filtered_data_list), filtered_data_list[0:5])

    # Create the plot
    plot_data(pressure_data_list, filtered_data_list)

if __name__ == "__main__":
    main()
