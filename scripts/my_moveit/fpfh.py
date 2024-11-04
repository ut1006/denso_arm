from pathlib import Path
import open3d as o3d
import copy
import numpy as np

def draw_registration_result(source, target, transformation):
    # Visualize the alignment result without saving
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target],
                                      zoom=0.5,
                                      front=[-0.2458, -0.8088, 0.5342],
                                      lookat=[1.7745, 2.2305, 0.9787],
                                      up=[0.3109, -0.5878, -0.7468])

def align_with_targets(source_file, target_files):
    source = o3d.io.read_point_cloud(source_file)
    current_transformation = np.identity(4)

    for target_file in target_files:
        print(f"\nAligning with target: {target_file}")
        
        target = o3d.io.read_point_cloud(target_file)
        voxel_radius = [0.04, 0.02, 0.01]
        max_iter = [50, 30, 14]

        for scale in range(3):
            iter = max_iter[scale]
            radius = voxel_radius[scale]

            # Downsample and estimate normals
            source_down = source.voxel_down_sample(radius)
            target_down = target.voxel_down_sample(radius)
            source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
            target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

            # Perform colored ICP
            result_icp = o3d.pipelines.registration.registration_colored_icp(
                source_down, target_down, radius, current_transformation,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=iter))
            current_transformation = result_icp.transformation

            print(result_icp)

        draw_registration_result(source, target, current_transformation)

    # Save the final transformed source point cloud after all alignments
    source.transform(current_transformation)
    source_path = Path(source_file)
    final_filename = source_path.stem + "_final_colored_icp.ply"
    final_path = source_path.with_name(final_filename)
    o3d.io.write_point_cloud(str(final_path), source)
    print("Final transformed source saved as:", final_filename)

# Define source and multiple targets
source_path = "output/0013/0013_transformed.ply"
target_paths = ["output/0014/0014_transformed.ply", "output/0015/0015_transformed.ply"]

# Align the source with each target
align_with_targets(source_path, target_paths)
