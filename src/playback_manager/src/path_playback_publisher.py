#!/usr/bin/env python
"""
Small Doc:

・前提:
行きの分のcsvデータはすでにtoward_csv内に保存してある状態であり、かつそのデータをplaybackするために各座標データをinvertedしたplayback_csvが保存されてある状態。

・Playbackについて:
行きはユーザーが入力した制御コマンドとその時のgps,imu,odometry,encorder_odometryを保存し、帰りのときは保存されたデータを逆順で読み込むことによって行ってきた道をそのままたどって帰るような自立移動走行方法の一つ。
今回、プレイバックは以下のように行うことにする。
1. joystickは、そのまま逆のデータを流し込んでplaybackを行う。
2. IMUはプレイバックのためのOdometryとして利用しない。その代わり、他のセンサーでプレイバックを行う際のロボットの進行方向のデータとして利用する。
3. GPS,Odometryセンサー, Encorderセンサーの3つはPlaybackのオドメトリとして利用する。

・PlaybackNodeの制御フロー:
----0. Playbackを行う際、ユーザーにどのセンサーでプレイバックさせるか引数で指定させる。
1. inved_csv内のgps_inv,odometry_inv,encorder_odometry_invの3つのうち、"0."で指定されたcsvのいずれかをloadする。
LOOP UNTIL(REACH_GOAL_POSITION)
    2. 現在のx, y座標, IMUを取得する。
    3. 現在のx, y座標と保存されているx, y座標を比べて、"何度ずれているか"と"どのくらい離れているか"を計算する。
    3-1. 現在、inved_csvの最後の座標を考慮しており、かつ角度のずれが、指定したしきい値以下になっているー>return REACH_GOAL_POSITION
    4. IMUで角度のずれを、指定したしきい値以下の角度になるまでcmd_velにpublishしてボディを回転させる。
    5. 離れている距離分だけ大きい値（-1.0~1.0）をcmd_velにpublishする（PID）。
    (すべてのセンサーの情報をcsv_playbackに保存する。)
"""

import tf
import rospy
import csv
import argparse
from math import atan2, sqrt, pow, pi
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs import GPSRAW
from path_estimation.generate_inved_coord import reverse_csv 

class PlaybackNode:
    def __init__(self, sensor_type="odom"):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz
        self.sensor_type = sensor_type

        # Load playback data from CSV based on sensor_type
        self.path = self.load_inv_path_from_csv(sensor_type)
        self.current_index = 0

        self.goal_tolerance = 0.1  # Goal tolerance in meters
        self.angle_tolerance = 0.1  # Angle tolerance in radians

        # Current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0

        self.vec = [0, 0]
        input_csv_path = '/home/cvl/ros_whill_ws/path_estimation/csv_toward/odom_data.csv'
        output_csv_path = '/home/cvl/ros_whill_ws/path_estimation/csv_toward/inv_odom_data.csv'
        reverse_csv(input_csv_path, output_csv_path)

    def load_inv_path_from_csv(self, sensor_type):
        file_name = '/home/cvl/ros_whill_ws/path_estimation/inv_csv/' + sensor_type + '_inv.csv'
        path = []
        with open(file_name, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                # CSVフォーマットに合わせてデータを取得（例: timestamp, x, y）
                path.append([float(row[0]), float(row[1]), float(row[2])])

        return path

    def control_loop(self):
        Th = self.goal_tolerance
        if self.current_index >= len(self.path):
            rospy.loginfo("Playback complete")
            return

        target = self.path[self.current_index]
        target_x, target_y = target[1], target[2]

        dx = self.current_x - target_x
        dy = self.current_y - target_y

        vec = [float((dx > Th) - (dx < Th)), float((dy > Th) - (dy < Th))]
        
        while not (vec[0] and vec[1]):
            twist = Twist()
            twist.linear.x, twist.linear.y = vec

            self.pub_cmd_vel.publish(twist)
            dx = self.current_x - target_x
            dy = self.current_y - target_y

            vec = [lambda dx, Th: (dx > Th) - (dx < Th), lambda dy, Th: (dy > Th) - (dy < Th)]
            
        else:
            self.current_index += 1  # 次の目標位置へ

        # 現在のセンサー情報を保存（例: playback_csvに保存）
        self.save_current_data()

    def save_current_data(self):
        # 現在のセンサー情報をCSVに保存する処理
        pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Playback node for GPS, odometry, or encoder data')
    parser.add_argument('sensor_type', choices=['gps', 'odom_sensor', 'odom'], help='Type of sensor data to playback')
    args = parser.parse_args()
    try:
        node = PlaybackNode(args.sensor_type)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
