import matplotlib.pyplot as plt
import csv
import math
from config import TOWARD_CSV_DIR_PATH, BACKUP_CSV_DIR_PATH, INVED_CSV_DIR_PATH 

# CSVファイルからデータを読み取る
x_datas = []
y_datas = []
time_datas = []

# file_name = INVED_CSV_DIR_PATH + 'inv_encoder_data.csv'
# file_name = TOWARD_CSV_DIR_PATH + "encoder_data.csv"
file_name = "/home/cvl/ros_whill_ws/path_estimation/csv_backup/inv_encoder_8_30.csv"
# file_name = BACKUP_CSV_DIR_PATH + "encoder_data_out_side.csv"

with open(file_name, 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        time_datas.append(float(row[0]))
        x_datas.append(-float(row[2]))
        y_datas.append(float(row[1]))

# 矢印をプロットする
plt.figure()
# plt.quiver(
#     x_datas[:-1], y_datas[:-1],  # 矢印の開始点 (x, y)
#     [x2 - x1 for x1, x2 in zip(x_datas[:-1], x_datas[1:])],  # 矢印のx成分
#     [y2 - y1 for y1, y2 in zip(y_datas[:-1], y_datas[1:])],  # 矢印のy成分
#     angles='xy', scale_units='xy', scale=1, color='b', width=0.003
# )
plt.scatter(x_datas[0], y_datas[0], color='r', s=50, label='Start')  # 開始点
plt.scatter(x_datas[1:-2], y_datas[1:-2], color='b', s=1, label='way')  # 開始点
plt.scatter(x_datas[-1], y_datas[-1], color='k', s=50, label='End')  # 終点
print("Difference distance from origin", math.sqrt(pow(x_datas[-1] - x_datas[0], 2) + pow(y_datas[-1] - y_datas[0], 2)))
plt.xlabel('X')
plt.ylabel('Y')
# plt.title('Playback root by Encorder')
plt.title('Point datas by Encorder')
plt.axis('equal')  # xとy軸の幅を同じにする
plt.grid(True)
plt.legend()
plt.show()
