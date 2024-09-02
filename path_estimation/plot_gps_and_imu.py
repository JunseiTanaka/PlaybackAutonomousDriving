import matplotlib.pyplot as plt
import csv
import numpy as np

# CSVファイルからGPSデータを読み取る
gps_x = []
gps_y = []
gps_time = []

gps_file_name = '/home/cvl/ros_whill_ws/path_estimation/csv_toward/gps_data.csv'

with open(gps_file_name, 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        gps_time.append(float(row[0]))
        gps_x.append(float(row[2]))
        gps_y.append(float(row[1]))

# CSVファイルからIMUデータを読み取る
imu_accel_x = []
imu_accel_y = []
imu_time = []

imu_file_name = '/home/cvl/ros_whill_ws/path_estimation/csv_toward/imu_data.csv'

with open(imu_file_name, 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        imu_time.append(float(row[0]))
        imu_accel_x.append(float(row[1]))
        imu_accel_y.append(float(row[2]))

# IMUデータとGPSデータのタイムスタンプを合わせる
def find_nearest_time_index(time_list, target_time):
    return min(range(len(time_list)), key=lambda i: abs(time_list[i] - target_time))

matched_imu_indices = [find_nearest_time_index(imu_time, t) for t in gps_time]

# GPSデータをプロットする
plt.figure()
plt.scatter(gps_x, gps_y, color='b', label='GPS Points')  # GPSの点をプロット

# IMUデータのベクトルをプロットする
scale_factor = 5000  # スケールファクターを調整
for i, idx in enumerate(matched_imu_indices):
    plt.quiver(
        gps_x[i], gps_y[i],  # 矢印の開始点 (x, y)
        imu_accel_x[idx] * scale_factor, imu_accel_y[idx] * scale_factor,  # 矢印のx成分, y成分
        angles='xy', scale_units='xy', scale=1, color='r', width=0.003
    )

plt.scatter(gps_x[0], gps_y[0], color='r', s=50, label='Start')  # 開始点
plt.scatter(gps_x[-1], gps_y[-1], color='k', s=50, label='End')  # 終点

plt.xlabel('X')
plt.ylabel('Y')
plt.title('GPS Points with IMU Vectors')
plt.axis('equal')  # xとy軸の幅を同じにする
plt.grid(True)
plt.legend()
plt.show()
