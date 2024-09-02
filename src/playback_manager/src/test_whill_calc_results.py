import rospy
import math
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import tf
import csv
import numpy as np


class WhillAutonomousNavigation:
    def __init__(self, x, y, whill_norm_angle, goals, output_csv):
        self.x = x
        self.y = y
        self.whill_norm_angle = whill_norm_angle
        self.theta = 0
        self.goals = goals
        self.goal_index = 0
        self.x_goal, self.y_goal, _ = self.goals[self.goal_index]

        self.ROTATE_THRESHOLD = 0.03
        self.GOAL_THRESHOLD = 0.1
        self.joy = Joy()
        self.joy.axes = [0, 0]
        
        self.output_csv_file = output_csv

        rospy.init_node('whill_autonomous_navigation', anonymous=True)
        rospy.Subscriber('/whill/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/whill/controller/joy', Joy, queue_size=10)
        self.rate = rospy.Rate(10)
        
    
    def odom_callback(self, msg):
        # self.whill_norm_angle = msg.pose.pose.orientation.z
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        # degree = self.whill_norm_angle2degree_converter(self.whill_norm_angle)
        # self.theta = self.deg2theta(degree)
        self.theta = msg.pose.pose.orientation.z * math.pi
        self.theta = self.angle_converter(self.theta)
        # if self.theta < 0:
        #     self.theta = self.theta + math.pi
        # else:
        #     self.theta = self.theta - math.pi

        self.y = msg.pose.pose.position.x       
        self.x = msg.pose.pose.position.y

        # print("x:", self.x,"y:", self.y, "goal x:", self.x_goal,"goal y:", self.y_goal )
    
    def angle_converter(self, angle):
        if angle < 0:
            return 2*math.pi + angle
        else:
            return angle
    
    def cal_rotate_ang(self, goal_ang, current_ang):
        d_theta = goal_ang - current_ang
        if d_theta >= 0 and abs(d_theta) >= math.pi:
            return -(2*math.pi-d_theta)
        
        elif d_theta >= 0 and abs(d_theta) <= math.pi:
            return d_theta
        
        elif d_theta <= 0 and abs(d_theta) >= math.pi:
            return 2*math.pi+d_theta
        
        elif d_theta <=0 and abs(d_theta) <= math.pi:
            return d_theta
        else:
            print("something worng")
            return 0

    def calculate_directions(self):
        """ 現在の位置から目標位置までの距離と角度を計算する """
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        distance = np.sqrt(dx**2 + dy**2)
        target_angle =  np.arctan2(dx, dy)
        target_angle = self.angle_converter(target_angle)
        # if target_angle < 0:
        #     target_angle = target_angle + math.pi
        # else:
        #     target_angle = target_angle - math.pi

        # rotate_angle = (target_angle - self.theta + math.pi) % (math.pi) 
        rotate_angle = self.cal_rotate_ang(target_angle, self.theta) 
        # print("distance", distance, "target_angle", target_angle, "theta", self.theta, "rotate_angle", rotate_angle)
        return distance, target_angle, rotate_angle

    def should_rotate(self, rotate_angle):
        """ 回転が必要かどうかを判断する """ 

        return np.abs(rotate_angle) > self.ROTATE_THRESHOLD and not abs(rotate_angle) > (math.pi*2-0.5)

    def rotate(self, rotate_angle, target_angle, step=0.5):
        """ 実際にロボットの向きを調整する """
        if rotate_angle >= 0:
            self.joy.axes = [step, 0]
        
        elif rotate_angle <= 0:
            self.joy.axes = [-step, 0]
        
        else:
            self.joy.axes = [0, 0]
            
        self.cmd_vel_pub.publish(self.joy)

    def move_forward(self, step=0.5):
        """ ロボットを前進させる """
        self.joy.axes = [0, step]
        self.cmd_vel_pub.publish(self.joy)

    def at_goal(self):
        """ 現在の位置が目標に到達しているかを確認する """
        distance, _, _ = self.calculate_directions()
        return distance < self.GOAL_THRESHOLD

    def update_goal(self):
        """ ゴールを更新する """
        if self.at_goal():
            self.goal_index += 1
            if self.goal_index < len(self.goals):
                self.x_goal, self.y_goal, _ = self.goals[self.goal_index]
                print(f"###################Next goal###################: ({self.x_goal}, {self.y_goal})")
                return True
            else:
                return False  # すべての目標をクリアした
        return True

    def update_position(self):
        """ ロボットの位置を更新する """
        while not rospy.is_shutdown():
            # print(self.x, self.y, self.theta)
            distance, target_angle, rotate_angle = self.calculate_directions()
            print("distance", distance, "target_angle", target_angle, "theta", self.theta, "rotate_angle", rotate_angle,"\n", "x", self.x, "next x", self.x_goal,  "y", self.y, "next y", self.y_goal)

            if not self.at_goal():
                if self.should_rotate(rotate_angle):
                    self.rotate(rotate_angle, target_angle)
                    print("rotate")
                else:
                    self.move_forward()
                    print("forward")
            
            next = self.update_goal()
            self.save_to_csv(distance, target_angle, rotate_angle)
            if not next:
                print("Finish")
                exit()

            self.rate.sleep()
    def run(self):
        while not rospy.is_shutdown():
            print(self.angle_converter(self.theta))
            self.rate.sleep()

    def save_to_csv(self, distance, target_angle, rotate_angle):
        """ Save the calculated values to the CSV file """
        with open(self.output_csv_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([rospy.get_time(), distance, target_angle, self.theta, rotate_angle, self.x, self.x_goal, self.y, self.y_goal])


def extract_goals(input_file):
    goals = []
    with open(input_file, 'r', newline='') as f:
        reader = csv.reader(f) 
        for row in reader:
            if len(row) >= 3:
                x = float(row[2])
                y = float(row[1])
                z = float(row[3])
                goals.append([x, y, z])
    
    return goals
    

if __name__ == '__main__':
    try:
        input_file = '/home/cvl/ros_whill_ws/path_estimation/inv_csv/inv_encoder_data.csv'
        output_csv = '/home/cvl/ros_whill_ws/path_estimation/csv_playback/playback_encoder_data.csv'
        goals = extract_goals(input_file)
        
        robot = WhillAutonomousNavigation(x=goals[0][0], y=goals[0][1], whill_norm_angle=goals[0][2], goals=goals, output_csv=output_csv)
        robot.run()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
