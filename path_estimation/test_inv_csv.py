import csv

# 入力ファイルと出力ファイルのパスを指定
input_file = 'input.csv'
output_file = 'output.csv'

# CSVファイルの行をリストとして読み込む
with open(input_file, 'r', newline='') as f:
    reader = csv.reader(f)
    rows = list(reader)

# 行の順番を逆にする
rows.reverse()

# 逆順になった行を新しいCSVファイルに書き込む
with open(output_file, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(rows)


print("列の順番を反転したデータが", output_file, "に保存されました。")
