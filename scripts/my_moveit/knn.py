import open3d as o3d
import numpy as np
from sklearn.neighbors import NearestNeighbors

# Step 1: Load the Point Cloud
pcd = o3d.io.read_point_cloud("output/0013/cleaned_point_cloud.ply")
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)  # Get the original colors

# Step 2: Compute the Nearest Neighbors
nbrs = NearestNeighbors(n_neighbors=2)  # Use n_neighbors=2 to find the nearest neighbor
nbrs.fit(points)
distances, indices = nbrs.kneighbors(points)

# distances[:, 1] gives the distances to the nearest neighbor
nearest_distances = distances[:, 1]

# Step 3: Filter Points
threshold_distance = 0.02
inlier_mask = nearest_distances <= threshold_distance

# Create a new point cloud for visualization
filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(points)

# Change colors based on inlier/outlier status
filtered_colors = colors.copy()
filtered_colors[~inlier_mask] = [1, 0, 0]  # Set outliers to red

filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

# Step 4: Visualize the Modified Point Cloud
o3d.visualization.draw_geometries([filtered_pcd], window_name="Filtered Point Cloud with Outliers in Red")
