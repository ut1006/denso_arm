import os
import csv
import glob
# カレントディレクトリを指定
current_dir = os.getcwd()

# サブディレクトリから"tf"が含まれるCSVファイルをすべて取得
for file_path in glob.glob(os.path.join(current_dir, '**', 'tf*.csv'), recursive=True):
    try:
        # ファイルの読み込み
        with open(file_path, mode='r') as file:
            reader = list(csv.reader(file))
            if len(reader) < 2:
                print(f"File {file_path} does not have enough rows.")
                continue

            # 2行目の最初の3つの要素を1000倍
            row_data = reader[1]
            row_data[0] = str(float(row_data[0]) * 1000)  # Translation X
            row_data[1] = str(float(row_data[1]) * 1000)  # Translation Y
            row_data[2] = str(float(row_data[2]) * 1000)  # Translation Z
            reader[1] = row_data  # 更新したデータで2行目を書き換え

        # 上書き保存
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(reader)
        
        print(f"Updated: {file_path}")

    except Exception as e:
        print(f"Error processing {file_path}: {e}")
