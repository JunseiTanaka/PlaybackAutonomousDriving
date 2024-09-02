import matplotlib.pyplot as plt
import csv
import math
from config import TOWARD_CSV_DIR_PATH

# CSVファイルからデータを読み取る
x_datas = []
y_datas = []
time_datas = []

file_name = TOWARD_CSV_DIR_PATH + 'odom_sensor_data.csv'
# odom_data.csvファイルを読み取る
with open(file_name, 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        #time_datas.append(float(row[0]))
        x_datas.append(float(row[1]))
        y_datas.append(-float(row[2]))

# 矢印をプロットする
plt.figure()
plt.quiver(
    x_datas[:-1], y_datas[:-1],  # 矢印の開始点 (x, y)
    [x2 - x1 for x1, x2 in zip(x_datas[:-1], x_datas[1:])],  # 矢印のx成分
    [y2 - y1 for y1, y2 in zip(y_datas[:-1], y_datas[1:])],  # 矢印のy成分
    angles='xy', scale_units='xy', scale=1, color='b', width=0.003
)
plt.scatter(x_datas[0], y_datas[0], color='r', s=50, label='Start')  # 開始点
plt.scatter(x_datas[-1], y_datas[-1], color='k', s=50, label='End')  # 終点
print("Difference distance from origin", math.sqrt(pow(x_datas[-1] - x_datas[0], 2) + pow(y_datas[-1] - y_datas[0], 2)))
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Odom sensor Data Outdoor')
plt.axis('equal')  # xとy軸の幅を同じにする
plt.grid(True)
plt.legend()
plt.show()
