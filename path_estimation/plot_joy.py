import matplotlib.pyplot as plt
import csv
import math
from config import TOWARD_CSV_DIR_PATH

file_name=TOWARD_CSV_DIR_PATH + 'joy_data.csv'

x_datas = []
y_datas = []
time_datas = []
x_data_sum = 0
y_data_sum = 0

x_data_sums = []
y_data_sums = []
FIRST_LOOP = True
start_time = 0.0
before_time = 0.0

velocity = 0.83 # 3 km/h to m/s --> 3/3.6 = 0.83 m/s
ang_velocity = 0.83 # 3km/h to m/s --> 3/3.6 = 0.83 m/s

def cmd_vel2_xy_coord(cmd_x, cmd_y, d_time):
    theta = cmd_y * ang_velocity * d_time
    d = cmd_x * velocity * d_time

    x = d * math.cos(theta)
    y = d * math.sin(theta)

    return (x,y) 

with open(file_name, 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        
        if FIRST_LOOP:
            sec_len = len(row[0])
            start_time_sec = row[0]
            start_time_dec = row[1]
            current_time = int(start_time_sec) + int(start_time_dec) * math.pow(0.1, sec_len)
            before_time = current_time
            FIRST_LOOP = False 
            continue
        
        start_time_sec = row[0]
        start_time_dec = row[1]
        current_time = int(start_time_sec) + int(start_time_dec) * math.pow(0.1, sec_len)
        d_time = current_time - before_time
        cmd_x = float(row[3])
        cmd_y = float(row[2])

        x, y = cmd_vel2_xy_coord(cmd_x, cmd_y, d_time)
        #print("x, y, d_t", x, y, d_time)
        before_time = current_time

        x_data_sum += x
        x_data_sums.append(x_data_sum)
        y_data_sum += y
        y_data_sums.append(y_data_sum)
        print(x_data_sum, y_data_sum)

# 矢印をプロットする
plt.figure()
plt.quiver(
    x_data_sums[:-1], y_data_sums[:-1],  # 矢印の開始点 (x, y)

    [x2 - x1 for x1, x2 in zip(x_data_sums[:-1], x_data_sums[1:])],  # 矢印のx成分
    [y2 - y1 for y1, y2 in zip(y_data_sums[:-1], y_data_sums[1:])],  # 矢印のy成分
    angles='xy', scale_units='xy', scale=1, color='b', width=0.003

)
plt.scatter(x_data_sums[0], y_data_sums[0], color='r', s=50, label='Start')  # 開始点
plt.scatter(x_data_sums[-1], y_data_sums[-1], color='k', s=50, label='End')  # 終点
print("Difference distance from origin", math.sqrt(pow(x_data_sums[-1] - x_data_sums[0], 2)+ pow(y_data_sums[-1] - y_data_sums[0],2)))
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Joy Data')
plt.axis('equal')  # xとy軸の幅を同じにする
plt.grid(True)
plt.show()
