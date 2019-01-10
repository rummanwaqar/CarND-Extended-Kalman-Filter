import matplotlib.pyplot as plt
import csv
import sys
import math
import os

def read_input_file(file_name):
    ground_truth = []
    lidar = []
    radar = []
    with open(file_name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter='\t')
        for row in csv_reader:
            sensor_type = row[0]
            data = list(map(float, row[1:]))

            # parse ground truth data
            ground_truth.append({
                'px': data[-6],
                'py': data[-5],
                'vx': data[-4],
                'vy': data[-3]
            })

            # parse lidar data
            if sensor_type == 'L':
                lidar.append({
                    'x': data[0],
                    'y': data[1]
                })

            # parse radar data
            if sensor_type == 'R':
                rho = data[0]
                phi = data[1]
                radar.append({
                    'x': math.cos(phi) * rho,
                    'y': math.sin(phi) * rho
                })

    return lidar, radar, ground_truth

def read_output_file(file_name):
    estimates = []
    with open(file_name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter='\t')
        for row in csv_reader:
            data = list(map(float, row))
            estimates.append({
                'px': data[0],
                'py': data[1],
                'vx': data[2],
                'vy': data[3]
            })
    return estimates

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("python plot.py input_data.txt output_data.txt")
        exit(0)

    lidar, radar, ground_truth = read_input_file(sys.argv[1])
    estimates = read_output_file(sys.argv[2])

    plt.figure(0)
    plt.title('EKF Visualization')

    # plot lidar
    plt.plot([d['x'] for d in lidar],
             [d['y'] for d in lidar],
             'g*')

    # plot radar
    plt.plot([d['x'] for d in radar],
             [d['y'] for d in radar],
             'bx')

    # plot ground truth
    plt.plot([d['px'] for d in ground_truth],
             [d['py'] for d in ground_truth],
             'r-')

    plt.plot([d['px'] for d in estimates],
             [d['py'] for d in estimates],
             'c-')

    plt.legend(['lidar', 'radar', 'ground truth', 'estimate'])

    plt.show()
