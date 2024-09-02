import matplotlib.pyplot as plt
import csv
import math
from config import TOWARD_CSV_DIR_PATH, BACKUP_CSV_DIR_PATH

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
        x_datas.append(float(row[1])*0.0254)
        y_datas.append(-float(row[2])*0.0254)

plt.quiver(
    x_datas[:-1], y_datas[:-1],  # 矢印の開始点 (x, y)
    [x2 - x1 for x1, x2 in zip(x_datas[:-1], x_datas[1:])],  # 矢印のx成分
    [y2 - y1 for y1, y2 in zip(y_datas[:-1], y_datas[1:])],  # 矢印のy成分
    angles='xy', scale_units='xy', scale=1, color='b', width=0.003, label="odom sensor"
)
plt.scatter(x_datas[0], y_datas[0], color='r', s=50, label='odom Start')  # 開始点
plt.scatter(x_datas[-1], y_datas[-1], color='k', s=50, label='odom End')  # 終点
print("Difference distance from origin", math.sqrt(pow(x_datas[-1] - x_datas[0], 2) + pow(y_datas[-1] - y_datas[0], 2)))
plt.axis('equal')  # xとy軸の幅を同じにする
plt.grid(True)
plt.legend()

# CSVファイルからデータを読み取る
encoder_x_datas = []
encoder_y_datas = []
encoder_time_datas = []

encoder_file_name = TOWARD_CSV_DIR_PATH + 'odom_data.csv'

with open(encoder_file_name, 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        encoder_time_datas.append(float(row[0]))
        encoder_x_datas.append(-float(row[2]))
        encoder_y_datas.append(float(row[1]))

plt.quiver(
    encoder_x_datas[:-1], encoder_y_datas[:-1],  # 矢印の開始点 (x, y)
    [x2 - x1 for x1, x2 in zip(encoder_x_datas[:-1], encoder_x_datas[1:])],  # 矢印のx成分
    [y2 - y1 for y1, y2 in zip(encoder_y_datas[:-1], encoder_y_datas[1:])],  # 矢印のy成分
    angles='xy', scale_units='xy', scale=1, color='g', width=0.003, label="encoder sensor"
)
# Correct path rectangle
rect = plt.Rectangle((0, 0), 8.7, 15, linewidth=3, edgecolor='black', facecolor='none', label='Correct path')
plt.gca().add_patch(rect)
plt.scatter(encoder_x_datas[0], encoder_y_datas[0], color='r', s=50, label='encoder Start')  # 開始点
plt.scatter(encoder_x_datas[-1], encoder_y_datas[-1], color='k', s=50, label='encoder End')  # 終点
print("Difference distance from origin", math.sqrt(pow(encoder_x_datas[-1] - encoder_x_datas[0], 2) + pow(encoder_y_datas[-1] - encoder_y_datas[0], 2)))
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Encorder&Odom Datas Indoor')
plt.axis('equal')  # xとy軸の幅を同じにする
plt.grid(True)
plt.legend(loc='upper left')
plt.show()
