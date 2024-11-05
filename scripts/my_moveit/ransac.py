import open3d as o3d
import numpy as np

# 点群の読み込み
pcd = o3d.io.read_point_cloud("output/0013/cleaned_point_cloud.ply")
# 引数を設定して、平面を推定
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
# 平面モデルの係数を出力
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# インライアの点を抽出して色を付ける
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])

# 平面以外の点を抽出
outlier_cloud = pcd.select_by_index(inliers, invert=True)

# 可視化
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                  zoom=0.8,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])

