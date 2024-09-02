import rospy
import math
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import tf
import csv
from std_msgs.msg import Float64
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
        self.next = True

        self.ROTATE_THRESHOLD = 0.01
        self.LOOK_AHEAD_DISTANCE = 0.01
        self.GOAL_THREASHOLD = 0.001
        self.joy = Joy()
        self.joy.axes = [0, 0]
        
        self.output_csv_file = output_csv

        rospy.init_node('whill_autonomous_navigation', anonymous=True)
        
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.hdg_callback)

        self.cmd_vel_pub = rospy.Publisher('/whill/controller/joy', Joy, queue_size=10)
        self.rate = rospy.Rate(10)
    
    def deg2theta(self, deg):
        theta =  deg * math.pi / 180
        # print(deg, theta)
        return theta
    
    def gps_callback(self, msg:NavSatFix):
        self.x = msg.latitude
        self.y = msg.longitude
        # print("x:", self.x,"y:", self.y, "goal x:", self.x_goal,"goal y:", self.y_goal )
    
    def hdg_callback(self, msg:Float64):
        self.theta = self.deg2theta(msg.data)

    def angle_converter(self, angle):
        if angle < 0:
            return 2*math.pi + angle
        else:
            return angle
    
    def calc_rotate_ang(self, goal_ang, current_ang):
        d_theta = goal_ang - current_ang
        if d_theta >= 0 and abs(d_theta) >= math.pi:
            print(1)
            return -(2*math.pi-d_theta)
        
        elif d_theta >= 0 and abs(d_theta) <= math.pi:
            print(2)
            return -d_theta
        
        elif d_theta <= 0 and abs(d_theta) >= math.pi:
            print(3)
            return 2*math.pi+d_theta
        
        elif d_theta <=0 and abs(d_theta) <= math.pi:
            print(4)
            return -d_theta
        
        else:
            print("something worng")
            return 0

    def calculate_directions(self):
        """ 現在の位置から目標位置までの距離と角度を計算する """
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        distance = np.sqrt(dx**2 + dy**2)
        target_angle =  np.arctan2(dy, dx)
        target_angle = self.angle_converter(target_angle)
        
        rotate_angle = self.calc_rotate_ang(target_angle, self.theta) 
        
        return distance, target_angle, rotate_angle

    def should_rotate(self, rotate_angle):
        """ 回転が必要かどうかを判断する """ 

        return np.abs(rotate_angle) > self.ROTATE_THRESHOLD 

    def rotate(self, rotate_angle, target_angle, step=1.0):
        """ 実際にロボットの向きを調整する """
        Kp = 0.5
        self.joy.axes[0] = rotate_angle*Kp
    
    def should_move_forward(self, rotate_angle):
        return abs(rotate_angle) < math.pi/2

    def move_forward(self, distance, step=0.5):
        """ ロボットを前進させる """
        Kp = 15000
        if distance < self.GOAL_THREASHOLD:
            self.joy.axes[1] = 0
        else:
            self.joy.axes[1] =  distance*Kp

    def less_than_lookahead_distance(self):
        """ 現在の位置が目標に到達しているかを確認する """
        distance, target_angle, rotate_angle = self.calculate_directions()
        return distance < self.LOOK_AHEAD_DISTANCE 
    
    def at_goal(self):
        distance, target_angle, rotate_angle = self.calculate_directions()
        return distance < self.GOAL_THREASHOLD and not self.should_rotate(rotate_angle)


    def update_goal(self):
        """ ゴールを更新する """
        if self.less_than_lookahead_distance():
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
            print("cmd",self.joy.axes,"distance", distance, "target_angle", target_angle, "theta", self.theta, "rotate_angle", rotate_angle,"\n",
                   "x", self.x, "next x", self.x_goal,  "y", self.y, "next y", self.y_goal)

            if not self.less_than_lookahead_distance():
                if self.should_rotate(rotate_angle):
                    self.rotate(rotate_angle, target_angle)
                    print("rotate")
                else:
                    rotate_angle = 0
                    self.rotate(rotate_angle, target_angle)
                
                if self.should_move_forward(rotate_angle):
                    self.move_forward(distance)
                    print("forward")

            else:
                if self.next: # もしnextがFalseだったらupdate_goalは計算する必要がないから
                    self.next = self.update_goal()
                        
            if not self.next: #最後
                if not self.at_goal() or self.should_rotate(rotate_angle):# 最後にrotateしてほしいから
                    if self.should_rotate(rotate_angle):
                        self.rotate(rotate_angle, target_angle)
                        print("last rotate")
                    
                    if self.should_move_forward(rotate_angle):
                        self.move_forward(distance)
                        print("last forward")
                else:
                    print("Finish")
                    exit()

            self.save_to_csv(distance, target_angle, rotate_angle)
            self.cmd_vel_pub.publish(self.joy)

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
                x = float(row[1])
                y = float(row[2])
                z = float(row[3])
                goals.append([x, y, z])
    
    return goals
    

if __name__ == '__main__':
    try:
        input_csv_file = '/home/cvl/ros_whill_ws/path_estimation/csv_toward/gps_data.csv'
        output_csv_file = '/home/cvl/ros_whill_ws/path_estimation/csv_playback/playback_gps_data.csv'

        goals = extract_goals(input_csv_file)
        
        robot = WhillAutonomousNavigation(x=goals[0][0], y=goals[0][1], whill_norm_angle=goals[0][2], goals=goals, output_csv=output_csv_file)
        robot.update_position()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
