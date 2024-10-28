import open3d as o3d
import argparse
import sys

def visualize_ply_point_cloud(ply_file_path):
    # PLYファイルを点群として読み込む
    point_cloud = o3d.io.read_point_cloud(ply_file_path)

    # 点群を可視化
    o3d.visualization.draw_geometries([point_cloud], 
                                      window_name="PLY Point Cloud",
                                      width=800, height=600)

if __name__ == "__main__":
    # コマンドライン引数の処理
    parser = argparse.ArgumentParser(description="PLYファイルの点群データを可視化するスクリプト")
    parser.add_argument("file", help="表示するPLYファイルのパス")
    
    # 引数の取得
    args = parser.parse_args()
    
    # ファイルパスが正しくない場合のエラーチェック
    try:
        visualize_ply_point_cloud(args.file)
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        sys.exit(1)
