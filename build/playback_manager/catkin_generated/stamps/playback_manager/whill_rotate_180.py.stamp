#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import tf

# グローバル変数
current_yaw = None
target_yaw = None
target_reached = False

def odom_callback(msg):
    global current_yaw

    # 現在のオリエンテーションを取得
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

    # 現在のyawを更新
    current_yaw = yaw

def rotate_to_target(yaw_goal):
    global current_yaw, target_yaw, target_reached

    # ノードの初期化
    rospy.init_node('whill_rotate_to_target', anonymous=True)
    
    # オドメトリ情報を購読
    rospy.Subscriber('/whill/odom', Odometry, odom_callback)
    
    # パブリッシャーの定義
    cmd_vel_pub = rospy.Publisher('/whill/controller/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10Hz
    twist = Twist()

    # 目標yawを設定
    target_yaw = yaw_goal

    while not rospy.is_shutdown():
        if current_yaw is not None:
            # 現在のyawと目標yawの差を計算
            yaw_error = target_yaw - current_yaw

            # -π ~ πの範囲に収める
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            # 目標に到達したかチェック
            if abs(yaw_error) < 0.01:
                target_reached = True
                twist.angular.z = 0.0
            else:
                # 回転速度を設定（目標に向かって回転）
                twist.angular.z = 0.5 * yaw_error / abs(yaw_error)  # 符号を付ける

            cmd_vel_pub.publish(twist)

            if target_reached:
                break

        rate.sleep()


if __name__ == '__main__':
    try:
        # 目標yaw角度をラジアン単位で設定
        target_yaw_angle = math.radians(0)  # 例として90度（π/2ラジアン）
        rotate_to_target(target_yaw_angle)

        
    except rospy.ROSInterruptException:
        pass
