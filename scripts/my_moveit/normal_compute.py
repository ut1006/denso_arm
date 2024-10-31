import open3d as o3d

# 点群の読み込み
pcd = o3d.io.read_point_cloud("0003_transformed.ply")

# 法線を推定
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)

# 法線方向を統一（optional）
pcd.orient_normals_consistent_tangent_plane(100)

# 結果を確認
o3d.visualization.draw_geometries([pcd], point_show_normal=True)

# 法線情報を含めた点群データを保存
o3d.io.write_point_cloud("0003_transformed_with_normals.ply", pcd)
