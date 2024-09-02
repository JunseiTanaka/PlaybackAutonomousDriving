import csv
import math
import sys
import argparse

class CoordinateProcessor:
    def __init__(self, input_file, output_file, threshold, sensor_type):
        self.input_file = input_file
        self.output_file = output_file
        self.threshold = threshold
        self.sensor_type = sensor_type

    def calculate_distance(self, p1, p2):
        """2つの座標間の距離を計算する"""
        if self.sensor_type == "gps":
            return math.sqrt((p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)
        
        return math.sqrt((p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)
        
    def read_coordinates(self):
        """CSVファイルから座標データを読み込む"""
        with open(self.input_file, 'r', newline='') as f:
            reader = csv.reader(f)
            return [list(map(float, row)) for row in reader]

    def reverse_rows(self, rows):
        """行の順番を逆にする"""
        rows.reverse()
        return rows

    def filter_coordinates(self, rows):
        """座標データをフィルタリングする"""
        filtered_rows = []
        i = 0
        idx1 = 0
        idx2 = 0
        cnt = 1

        while i < len(rows) - 1:
            current_row = rows[idx1]
            idx2 = idx1 + cnt
            next_row = rows[idx2]
            distance = self.calculate_distance(current_row, next_row)
            # print("idx1:", idx1, "idx2:", idx2, "current_row", current_row, "next_row:", next_row, "distance:", distance)
            if distance >= self.threshold:
                filtered_rows.append(current_row)
                idx1 = idx2
                cnt = 1
            else:
                cnt += 1
            i += 1
            

        # 最後の行を追加
        if len(rows) > 0:
            filtered_rows.append(rows[-1])

        return filtered_rows

    def write_coordinates(self, rows):
        """フィルタリングされた座標データをCSVファイルに書き込む"""
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(rows)

    def process(self):
        """全体の処理を実行する"""
        # 座標データを読み込み、行の順番を逆にする
        rows = self.read_coordinates()
        rows = self.reverse_rows(rows)
        
        # データをフィルタリングする
        filtered_rows = self.filter_coordinates(rows)
        
        # フィルタリングされたデータを新しいCSVファイルに書き込む
        self.write_coordinates(filtered_rows)
        print(f"座標データがフィルタリングされ、{self.output_file}に保存されました。")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', type=str, default="gps")
    args = parser.parse_args()

    if args.sensor == "encoder":
        threshold = 0.1
        input_csv_file = '/home/cvl/ros_whill_ws/path_estimation/inv_csv/inv_encoder_data.csv'
        output_csv_file = '/home/cvl/ros_whill_ws/path_estimation/csv_playback/playback_encoder_data.csv'
    
    elif args.sensor == "gps":
        threshold = 0.00002
        input_csv_file = '/home/cvl/ros_whill_ws/path_estimation/csv_toward/gps_data.csv'
        output_csv_file = '/home/cvl/ros_whill_ws/path_estimation/inv_csv/inv_gps_data.csv'
    
    else:
        print(f"sensor {args.sensor} is not defineded.")
        exit()
    

    processor = CoordinateProcessor(input_csv_file, output_csv_file, threshold, sensor_type=args.sensor)
    processor.process()
    
