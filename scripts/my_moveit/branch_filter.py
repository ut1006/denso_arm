import open3d as o3d
import numpy as np
import matplotlib.colors as mcolors

def load_point_cloud(file_path):
    # バイナリ形式のPLYファイルを読み込む
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def filter_brown_points(pcd):
    # 点群の色を取得
    colors = np.asarray(pcd.colors)

    # RGBAの場合はRGBに変換（アルファチャンネルを無視）
    if colors.shape[1] == 4:
        colors = colors[:, :3]  # RGBAからRGBに変換

    # RGBからHSVに変換
    rgb_normalized = colors  # Open3Dの色は0-1にスケールされていると仮定
    hsv_colors = mcolors.rgb_to_hsv(rgb_normalized)  # NumPy配列を使用して変換

    # HSVの最小値と最大値を表示
    hsv_min = hsv_colors.min(axis=0)
    hsv_max = hsv_colors.max(axis=0)
    print(f'HSV Min: {hsv_min}')
    print(f'HSV Max: {hsv_max}')

    # 色の範囲を定義（HSV）
    # 茶色の範囲（Hの値を0-1にスケール）
    brown_range1 = ((0/360, 0.0, 0.0), (50/360, 1.0, 1.0))  # 0~50度の範囲
    brown_range2 = ((310/360, 0.0, 0.0), (360/360, 1.0, 1.0))  # 310~360度の範囲

    # 茶色の範囲に該当する点だけを残す
    mask1 = (
        (brown_range1[0][0] <= hsv_colors[:, 0]) & (hsv_colors[:, 0] <= brown_range1[1][0]) &
        (brown_range1[0][1] <= hsv_colors[:, 1]) & (hsv_colors[:, 1] <= brown_range1[1][1]) &
        (brown_range1[0][2] <= hsv_colors[:, 2]) & (hsv_colors[:, 2] <= brown_range1[1][2])
    )
    
    mask2 = (
        (brown_range2[0][0] <= hsv_colors[:, 0]) & (hsv_colors[:, 0] <= brown_range2[1][0]) &
        (brown_range2[0][1] <= hsv_colors[:, 1]) & (hsv_colors[:, 1] <= brown_range2[1][1]) &
        (brown_range2[0][2] <= hsv_colors[:, 2]) & (hsv_colors[:, 2] <= brown_range2[1][2])
    )

    # X座標（奥行き）が1.4以下の点を選択するマスクを作成
    x_mask = np.asarray(pcd.points)[:, 0] <= 1.4

    # 最終的なマスクを作成
    final_mask = (mask1 | mask2) & x_mask  # 茶色のマスクとX座標のマスクを組み合わせる

    # 抽出した点群を作成
    filtered_points = np.asarray(pcd.points)[final_mask]
    filtered_colors = np.asarray(pcd.colors)[final_mask]
    
    # 新しい点群を作成
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
    
    # フィルタリング後の点数を表示
    print(f'Filtered Points Count: {len(filtered_points)}')
    
    return filtered_pcd

def remove_radius_outliers(pcd, nb_points=16, radius=0.05):
    print("Radius outlier removal")
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    print(f"Points remaining after radius outlier removal: {len(ind)}")
    return cl

def save_point_cloud(pcd, file_path):
    # 点群をPLYファイルとして保存
    o3d.io.write_point_cloud(file_path, pcd)
    print(f'Saved point cloud to {file_path}')

def visualize_point_cloud(pcd):
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    input_file = "output/0013/0013_transformed_final_colored_icp.ply"  # PLYファイルのパスを指定
    pcd = load_point_cloud(input_file)

    # 色と距離でマスクした点群をフィルタリング
    filtered_pcd = filter_brown_points(pcd)
    save_point_cloud(filtered_pcd, "output/0013/filtered_brown_points.ply")  # フィルタリングされた点群を保存

    # 半径外れ値除去を実行
    cleaned_pcd = remove_radius_outliers(filtered_pcd)
    save_point_cloud(cleaned_pcd, "output/0013/cleaned_point_cloud.ply")  # 外れ値除去後の点群を保存

    # 最終的な点群を可視化
    visualize_point_cloud(cleaned_pcd)
