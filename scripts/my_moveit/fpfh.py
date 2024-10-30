import sys
import copy
import numpy as np
import open3d as py3d
from open3d import registration_ransac_based_on_feature_matching as RANSAC
from open3d import registration_icp as ICP
from open3d import compute_fpfh_feature as FPFH


def show(model, scene, model_to_scene_trans=np.identity(4)):
    model_t = copy.deepcopy(model)
    scene_t = copy.deepcopy(scene)

    model_t.paint_uniform_color([1, 0, 0])
    scene_t.paint_uniform_color([0, 0, 1])

    model_t.transform(model_to_scene_trans)

    py3d.draw_geometries([model_t, scene_t])


model = py3d.read_point_cloud("output/0001/0001.ply")
scene = py3d.read_point_cloud("output/0003/0003.ply")
## PCLモデルを使うならこちら
#model = py3d.read_point_cloud("milk.pcd")
#scene = py3d.read_point_cloud("milk_cartoon_all_small_clorox.pcd")

# いろいろなサイズの元： model点群の1/10を基本サイズsizeにする
size = np.abs((model.get_max_bound() - model.get_min_bound())).max() / 10
kdt_n = py3d.KDTreeSearchParamHybrid(radius=size, max_nn=50)
kdt_f = py3d.KDTreeSearchParamHybrid(radius=size * 50, max_nn=50)

py3d.estimate_normals(model, kdt_n)
py3d.estimate_normals(scene, kdt_n)
show(model, scene)

# ダウンサンプリング
model_d = py3d.voxel_down_sample(model, size)
scene_d = py3d.voxel_down_sample(scene, size)
py3d.estimate_normals(model_d, kdt_n)
py3d.estimate_normals(scene_d, kdt_n)
show(model_d, scene_d)

# 特徴量計算
model_f = FPFH(model_d, kdt_f)
scene_f = FPFH(scene_d, kdt_f)

# 準備
checker = [py3d.CorrespondenceCheckerBasedOnEdgeLength(0.9),
           py3d.CorrespondenceCheckerBasedOnDistance(size * 2)]

est_ptp = py3d.TransformationEstimationPointToPoint()
est_ptpln = py3d.TransformationEstimationPointToPlane()

criteria = py3d.RANSACConvergenceCriteria(max_iteration=40000,
                                          max_validation=500)
# RANSACマッチング
result1 = RANSAC(model_d, scene_d,
                 model_f, scene_f,
                 max_correspondence_distance=size * 2,
                 estimation_method=est_ptp,
                 ransac_n=4,
                 checkers=checker,
                 criteria=criteria)
show(model_d, scene_d, result1.transformation)

# ICPで微修正
result2 = ICP(model, scene, size, result1.transformation, est_ptpln)
show(model, scene, result2.transformation)
