import rospy
import math
from sensor_msgs import Joy
from nav_msgs.msg import Odometry
import tf
import csv

class WhillAutonomousNavigation:
    def __init__(self):
        self.current_yaw = None
        self.target_yaw = None
        self.target_reached = False
        self.current_position = None
        self.current_orientation = None
        self.path = []

        rospy.init_node('whill_autonomous_navigation', anonymous=True)
        rospy.Subscriber('/whill/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/whill/controller/cmd_vel', Joy, queue_size=10)
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        self.current_position = msg.pose.pose.position  # ロボットの現在位置を更新する
        self.current_orientation = msg.pose.pose.orientation

    def rotate_to_target(self, yaw_goal):
        self.target_yaw = yaw_goal
        joy = Joy()

        while not self.target_reached and not rospy.is_shutdown():
            if self.current_yaw is not None:
                yaw_error = self.target_yaw - self.current_yaw
                rospy.loginfo('Yaw Error: {}'.format(yaw_error))

                if yaw_error > math.pi:
                    yaw_error -= 2 * math.pi
                elif yaw_error < -math.pi:
                    yaw_error += 2 * math.pi

                if abs(yaw_error) < 0.1:
                    self.target_reached = True
                    joy.angular.z = 0.0
                else:
                    joy.angular.z = 0.5 * yaw_error / abs(yaw_error)

                self.cmd_vel_pub.publish(joy)
            
            self.rate.sleep()

    def move_to_target(self, x_goal, y_goal):
        while self.path and not rospy.is_shutdown():
            if self.current_position is not None:  # 現在位置が取得できている場合のみ実行
                x_error = x_goal - self.current_position.x
                y_error = y_goal - self.current_position.y
                distance_error = math.sqrt(x_error**2 + y_error**2)
                rospy.loginfo('Distance Error: {}'.format(distance_error))

                if distance_error < 0.1:
                    self.path.pop(0)
                    if self.path:
                        next_point = self.path[0]
                        self.rotate_to_target(next_point[3])  # 次の目標のYAW角度を指定
                else:
                    angle_to_goal = math.atan2(y_error, x_error)
                    joy = Joy()
                    joy.linear.x = 0.2
                    joy.angular.z = 0.2 * (angle_to_goal - self.current_yaw)

                    self.cmd_vel_pub.publish(joy)

            self.rate.sleep()

    def load_inv_path_from_csv(self, file_name):
        with open(file_name, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.path.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])  # YAW情報が4番目の要素にあると仮定

    def start_navigation(self):
        if self.path:
            first_point = self.path[0]
            self.rotate_to_target(first_point[3])  # CSVファイルの角度を使用して回転

            while not rospy.is_shutdown():
                if self.target_reached:
                    self.move_to_target(first_point[1], first_point[2])
                    break

if __name__ == '__main__':
    try:
        nav = WhillAutonomousNavigation()
        nav.load_inv_path_from_csv('/home/cvl/ros_whill_ws/path_estimation/inv_csv/inv_odom_data.csv')
        nav.start_navigation()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
