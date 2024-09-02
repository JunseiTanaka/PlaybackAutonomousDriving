import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
from config import INVED_CSV_DIR_PATH

class Robot:
    def __init__(self, x, y, theta, goals):
        self.x = x
        self.y = y
        self.theta = theta
        self.goals = goals
        self.goal_index = 0
        self.x_goal, self.y_goal = self.goals[self.goal_index]

        self.ROTATE_THRESHOLD = 0.05
        self.GOAL_THRESHOLD = 0.01

    def calculate_directions(self):
        """ 現在の位置から目標位置までの距離と角度を計算する """
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        distance = np.sqrt(dx**2 + dy**2)
        target_angle = np.arctan2(dy, dx)
        rotate_angle = target_angle - self.theta
        return distance, target_angle, rotate_angle

    def should_rotate(self, rotate_angle):
        """ 回転が必要かどうかを判断する """
        return np.abs(rotate_angle) > self.ROTATE_THRESHOLD

    def rotate(self, rotate_angle, step=0.06):
        """ 実際にロボットの向きを調整する """
        self.theta += np.sign(rotate_angle) * step

    def move_forward(self, step=0.01):
        """ ロボットを前進させる """
        self.x += np.cos(self.theta) * step
        self.y += np.sin(self.theta) * step

    def at_goal(self):
        """ 現在の位置が目標に到達しているかを確認する """
        distance, _, _ = self.calculate_directions()
        return distance < self.GOAL_THRESHOLD

    def update_goal(self):
        """ ゴールを更新する """
        if self.at_goal():
            self.goal_index += 1
            if self.goal_index < len(self.goals):
                self.x_goal, self.y_goal = self.goals[self.goal_index]
                print(f"Next goal: ({self.x_goal}, {self.y_goal})")
                return True
            else:
                return False  # すべての目標をクリアした
        return True

    def update_position(self):
        """ ロボットの位置を更新する """
        _, _, rotate_angle = self.calculate_directions()

        if not self.at_goal():
            if self.should_rotate(rotate_angle):
                self.rotate(rotate_angle)
            else:
                self.move_forward()
        


class RobotAnimation:
    def __init__(self, robot):
        self.robot = robot
        self.fig, self.ax = plt.subplots()
        self.robot_marker, = self.ax.plot([], [], 'bo')  # ロボットを表す青い点
        self.arrow = None  # 矢印の初期化
        self.goal_markers, = self.ax.plot([g[0] for g in self.robot.goals], [g[1] for g in self.robot.goals], 'ro')  # ゴール地点を表す赤い点
        self.mergine = 10.0
        self.set_auto_limits()

    def set_auto_limits(self):
        """ データの範囲に基づいて表示範囲を自動設定し、アスペクト比を保つ """
        x_min = min(self.robot.x, self.robot.x_goal)
        x_max = max(self.robot.x, self.robot.x_goal)
        y_min = min(self.robot.y, self.robot.y_goal)
        y_max = max(self.robot.y, self.robot.y_goal)
        if x_max < 0:
            x_max_mergine = x_max + self.mergine
        else:
            x_max_mergine = x_max - self.mergine
        
        if y_max < 0:
            y_max_mergine = y_max - self.mergine
        else:
            y_max_mergine = y_max + self.mergine
        
        self.ax.set_xlim(x_min - self.mergine, x_max_mergine)
        self.ax.set_ylim(y_min - self.mergine, y_max_mergine)
        self.ax.set_aspect('equal', adjustable='box')

    def init_animation(self):
        self.robot_marker.set_data([], [])
        if self.arrow is not None:
            self.arrow.remove()  # もしarrowが存在していたら削除
        self.arrow = self.ax.arrow(self.robot.x, self.robot.y, np.cos(self.robot.theta), np.sin(self.robot.theta), 
                                   head_width=0.05, head_length=0.05, fc='blue', ec='blue')
        return self.robot_marker, self.arrow

    def update_animation(self, i):
        if not self.robot.update_goal():
            plt.close()  # すべての目標に到達したらアニメーションを終了
            exit()

        self.robot.update_position()

        # ロボットの位置と向きを更新
        self.robot_marker.set_data([self.robot.x], [self.robot.y])
        if self.arrow is not None:
            self.arrow.remove()
        self.arrow = self.ax.arrow(self.robot.x, self.robot.y, np.cos(self.robot.theta), np.sin(self.robot.theta), 
                                   head_width=0.05, head_length=0.05, fc='blue', ec='blue')
        
        return self.robot_marker, self.arrow

    def animate(self):
        anim = FuncAnimation(self.fig, self.update_animation, init_func=self.init_animation, frames=400, interval=100, blit=True)
        plt.show()

def extract_goals(input_file):
    goals = []
    
    with open(input_file, 'r', newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 3:
                x = float(row[2])
                y = float(row[1])
                goals.append((x, y))
    
    return goals


input_file = INVED_CSV_DIR_PATH + 'inv_encoder_data.csv'
goals = extract_goals(input_file)
print(goals)

robot = Robot(x=goals[0][0], y=goals[0][1], theta=0, goals=goals)

# アニメーションの実行
robot_animation = RobotAnimation(robot)
robot_animation.animate()
