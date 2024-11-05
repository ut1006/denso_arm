import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

# Step 1: Load the Point Cloud
pcd = o3d.io.read_point_cloud("output/0013/cleaned_point_cloud.ply")

# Step 2: Project Points onto the XY Plane
points = np.asarray(pcd.points)
xy_points = points[:, :2]  # Keep only x and y, discard z

# Step 3: Display the Projected Points
plt.scatter(xy_points[:, 0], xy_points[:, 1], s=1, color='blue', label="Projected Points (XY)")
plt.title("Projected 2D Points (XY Plane)")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.show()

# Step 4: Perform Clustering using DBSCAN
clustering = DBSCAN(eps=0.1, min_samples=5).fit(xy_points)
labels = clustering.labels_

# Step 5: Visualize Clusters
# Unique labels (-1 is for noise in DBSCAN)
unique_labels = set(labels)
colors = plt.cm.Spectral(np.linspace(0, 1, len(unique_labels)))

plt.figure()
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = (labels == k)
    xy = xy_points[class_member_mask]
    plt.scatter(xy[:, 0], xy[:, 1], s=10, color=col, label=f"Cluster {k}")

plt.title("Clustered 2D Projected Points (XY Plane)")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.show()
