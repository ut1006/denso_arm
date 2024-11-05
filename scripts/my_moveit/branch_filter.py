import open3d as o3d
import numpy as np
import matplotlib.colors as mcolors

def load_point_cloud(file_path):
    # Load the binary PLY file
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def filter_non_green_points(pcd):
    # Get point cloud colors
    colors = np.asarray(pcd.colors)

    # Convert RGBA to RGB if necessary
    if colors.shape[1] == 4:
        colors = colors[:, :3]

    # Convert RGB to HSV
    hsv_colors = mcolors.rgb_to_hsv(colors)

    # HSV range for green
    green_range = ((45/ 360, 0.3, 0.13), (135/ 360, 1.0, 0.745))

    # Mask to filter out green points
    green_mask = (
        (green_range[0][0] <= hsv_colors[:, 0]) & (hsv_colors[:, 0] <= green_range[1][0]) &
        (green_range[0][1] <= hsv_colors[:, 1]) & (hsv_colors[:, 1] <= green_range[1][1]) &
        (green_range[0][2] <= hsv_colors[:, 2]) & (hsv_colors[:, 2] <= green_range[1][2])
    )

    # Keep points that are NOT green
    non_green_mask = ~green_mask
    x_mask = np.asarray(pcd.points)[:, 0] <= 1.4  # Depth constraint

    # Final mask
    final_mask = non_green_mask & x_mask

    # Filtered points and colors
    filtered_points = np.asarray(pcd.points)[final_mask]
    filtered_colors = np.asarray(pcd.colors)[final_mask]

    # Create new filtered point cloud
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    print(f'Filtered Points Count: {len(filtered_points)}')
    return filtered_pcd

def remove_radius_outliers(pcd, nb_points=32, radius=0.05):
    print("Radius outlier removal")
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    print(f"Points remaining after radius outlier removal: {len(ind)}")
    return cl

def save_point_cloud(pcd, file_path):
    o3d.io.write_point_cloud(file_path, pcd)
    print(f'Saved point cloud to {file_path}')

def visualize_point_cloud(pcd):
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    input_file = "output/0013/0013_transformed_final_colored_icp.ply"
    pcd = load_point_cloud(input_file)

    # Filter non-green points
    filtered_pcd = filter_non_green_points(pcd)
    save_point_cloud(filtered_pcd, "output/0013/filtered_non_green_points.ply")

    # Apply radius outlier removal
    cleaned_pcd = remove_radius_outliers(filtered_pcd)
    save_point_cloud(cleaned_pcd, "output/0013/cleaned_point_cloud.ply")

    # Visualize final point cloud
    visualize_point_cloud(cleaned_pcd)
