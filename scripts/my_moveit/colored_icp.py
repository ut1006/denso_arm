import argparse
from pathlib import Path
import open3d as o3d
import copy
import numpy as np

def draw_registration_result_original_color(source, target, transformation, source_file):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    
    # 入力ファイル名の末尾に "_colored_icp" を追加して新しいファイル名を生成
    source_path = Path(source_file)  # 引数として受け取ったファイル名を使用
    new_filename = source_path.stem + "_colored_icp.ply"  # "_colored_icp" を追加
    new_path = source_path.with_name(new_filename)

    # 変換された点群を新しいファイル名で保存
    o3d.io.write_point_cloud(str(new_path), source_temp)
    print("save {}",new_filename)
    # 点群を視覚化
    o3d.visualization.draw_geometries([source_temp, target],
                                      zoom=0.5,
                                      front=[-0.2458, -0.8088, 0.5342],
                                      lookat=[1.7745, 2.2305, 0.9787],
                                      up=[0.3109, -0.5878, -0.7468])


def main(source_file, target_file):
    print("1. Load two point clouds and show initial pose")
    source = o3d.io.read_point_cloud(source_file)
    target = o3d.io.read_point_cloud(target_file)

    # draw initial alignment
    current_transformation = np.identity(4)
    draw_registration_result_original_color(source, target, current_transformation, source_file)

    # point to point ICP
    current_transformation = np.identity(4)
    print("2. Point-to-point ICP registration is applied on original point")
    print("   clouds to refine the alignment. Distance threshold 0.2.")
    result_icp = o3d.pipelines.registration.registration_icp(
        source, target, 0.2, current_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(result_icp)
    draw_registration_result_original_color(source, target,
                                            result_icp.transformation, source_file)

    # colored pointcloud registration
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [500, 30, 14]
    current_transformation = np.identity(4)
    print("3. Colored point cloud registration")
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        print("3-3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=iter))
        current_transformation = result_icp.transformation
        print(result_icp)

    # 最後に、最終的な変換を適用した点群を保存
    draw_registration_result_original_color(source, target,
                                            result_icp.transformation, source_file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Register two point clouds using ICP.')
    parser.add_argument('source', type=str, help='Path to the source point cloud (PLY file)')
    parser.add_argument('target', type=str, help='Path to the target point cloud (PLY file)')
    args = parser.parse_args()

    main(args.source, args.target)
