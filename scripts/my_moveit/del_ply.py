import os
import glob

# カレントディレクトリを指定
current_dir = os.getcwd()

# サブディレクトリから"transformed"が含まれるPLYファイルをすべて削除
for file_path in glob.glob(os.path.join(current_dir, '**', '*transformed*.ply'), recursive=True):
    try:
        os.remove(file_path)
        print(f"Deleted: {file_path}")
    except Exception as e:
        print(f"Error deleting {file_path}: {e}")
