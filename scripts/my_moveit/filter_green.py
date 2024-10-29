import open3d as o3d
import numpy as np
import colorsys  # HSVとRGBの変換に使用

# RGBからHSVへの変換関数
def rgb_to_hsv(rgb):
    return colorsys.rgb_to_hsv(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0)

# PLYファイルを読み込む
pcd = o3d.io.read_point_cloud("23.ply")

# 点群の色情報（Nx3の配列、各行はRGBの値）
colors = np.asarray(pcd.colors) * 255  # [0, 1] の範囲から [0, 255] にスケーリング

# RGBをHSVに変換
hsv_colors = np.array([rgb_to_hsv(color) for color in colors])

# 緑色の範囲をHSVで指定（H:色相、S:彩度、V:明度）
# 緑はH=0.25付近、Sは高め、Vは任意
lower_bound = np.array([0.25 - 0.15, 0.2, 0.2])  # H, S, Vの最小値
upper_bound = np.array([0.25 + 0.15, 1.0, 1.0])  # H, S, Vの最大値

# 緑色の範囲に合致する点のマスクを作成
mask = np.all((hsv_colors >= lower_bound) & (hsv_colors <= upper_bound), axis=1)

# 緑の範囲以外の点のみを残す
filtered_pcd0 = pcd.select_by_index(np.where(~mask)[0])

# 壁の白を除く
colors2 = np.asarray(filtered_pcd0.colors) * 255  # [0, 1] の範囲から [0, 255] にスケーリング

# RGBをHSVに変換
hsv_colors2 = np.array([rgb_to_hsv(color) for color in colors2])
lower_bound2 = np.array([0, 0.0, 0.4])  # H, S, Vの最小値
upper_bound2 = np.array([1.0, 0.5, 1.0])  # H, S, Vの最大値

# 白色の範囲に合致する点のマスクを作成
mask2 = np.all((hsv_colors2 >= lower_bound2) & (hsv_colors2 <= upper_bound2), axis=1)

# 白の範囲以外の点のみを残す
filtered_pcd = filtered_pcd0.select_by_index(np.where(~mask2)[0])

# 結果を表示
o3d.visualization.draw_geometries([filtered_pcd])

# 結果を保存
o3d.io.write_point_cloud("23filtered.ply", filtered_pcd)
