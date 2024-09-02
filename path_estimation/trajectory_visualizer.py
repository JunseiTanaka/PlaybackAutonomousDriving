import matplotlib.pyplot as plt
import numpy as np
import csv
import math


class TrajectoryVisualizer:
    def __init__(self, input_csv):
        self.input_csv = input_csv
        self.data = []

    def load_data(self):
        """ CSVファイルからデータを読み込む """
        with open(self.input_csv, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # ヘッダーをスキップ
            for row in reader:
                time = float(row[0])
                distance = float(row[1])
                target_angle = float(row[2]) 
                theta = float(row[3]) 
                rotate_angle = float(row[4])
                x = float(row[5])
                x_goal = float(row[6])
                y = float(row[7])
                y_goal = float(row[8])

                self.data.append([time, x, y, theta, x_goal, y_goal])

    def plot_trajectory(self):
        """ ロボットの進行軌跡と向きをプロットする """
        x = [row[1] for row in self.data]
        y = [row[2] for row in self.data]
        theta = [row[3] for row in self.data]
        x_goal = [row[4] for row in self.data]
        y_goal = [row[5] for row in self.data]

        plt.figure(figsize=(12, 12))
        plt.plot(y, x, marker='o', linestyle='-', label='Trajectory')
        plt.plot(y_goal, x_goal, marker='o', linestyle='-', label='Goal')

        # Vector size adjustment with scale parameter
        # plt.quiver(y[75], x[75], np.cos(theta[75]), np.sin(theta[75]), angles='xy', scale_units='xy', scale=5, color='r', label='Direction')
        # plt.quiver(y, x, np.cos(theta), np.sin(theta), angles='xy', scale_units='xy', scale=10, color='r', label='Direction')

        plt.xlabel('Y Position')
        plt.ylabel('X Position')
        plt.title('Robot Trajectory and Direction')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.show()

if __name__ == '__main__':
    # CSVファイルに保存
    # input_file = '/home/cvl/ros_whill_ws/path_estimation/csv_playback/playback_encoder_data.csv' 
    # input_file = "/home/cvl/ros_whill_ws/path_estimation/csv_backup/9_2_playback_encoder_data_toward_outside_straight.csv"
    input_file = "/home/cvl/ros_whill_ws/path_estimation/csv_playback/playback_gps_data.csv"
    visualizer = TrajectoryVisualizer(input_file)
    visualizer.load_data()
    visualizer.plot_trajectory()
