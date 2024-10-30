import open3d as o3d
import numpy as np

def load_point_cloud(file_path):
    # PLYファイルを読み込む
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def apply_icp(source_pcd, target_pcd, threshold=0.3):
    # 点群マッチング（ICP）を実行
    initial_transform = np.eye(4)  # 初期位置合わせ用の同次変換行列
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return reg_p2p.transformation, reg_p2p

def main(source_path, target_path):
    # 点群データの読み込み
    source_pcd = load_point_cloud(source_path)
    target_pcd = load_point_cloud(target_path)

    # ICPを適用して点群をマッチング
    transformation, reg_result = apply_icp(source_pcd, target_pcd)

    # 結果を表示
    print("Transformation Matrix:")
    print(transformation)

    # 点群の位置合わせを視覚化
    source_pcd.transform(transformation)
    o3d.visualization.draw_geometries([source_pcd, target_pcd])

# 使用例
source_path = 'output/0001/0001_transformed.ply'  # 1つ目のPLYファイル
target_path = 'output/0003/0003_transformed.ply'  # 2つ目のPLYファイル
main(source_path, target_path)
