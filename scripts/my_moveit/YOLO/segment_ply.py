import cv2  # OpenCV
from imageio.v3 import imread
import numpy as np
from pathlib import Path
from ultralytics import YOLO
from icecream import ic
import open3d as o3d

# icecream を無効化
ic.disable()

# Calibration parameters  
fx, fy, cx1, cy1 = 1400.6, 1400.6, 1103.65, 574.575
cx2 = 1102.84
baseline = 62.8749  # in millimeters

# 回転行列 (Z軸周りに-90度)
Rotation_Z_90 = np.array([
    [0, 1, 0],
    [-1, 0, 0],
    [0, 0, 1]
])

def load_image(image_file):
    """画像を読み込む関数"""
    image = imread(image_file)  # imageio を使って画像を読み込む
    ic(image.shape)  # 画像の形状を確認
    return image

def load_disp(disp_file):
    """深度マップを読み込む関数"""
    disp = np.load(disp_file)  # .npy ファイルから深度をロード
    ic(disp.shape)  # 深度マップの形状を確認
    return disp 

def overlay_all_masks(image, masks):
    """すべてのマスクを1つの画像に重ねる"""
    combined_mask = np.zeros_like(image)
    for i, mask in enumerate(masks):
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
        color = [0, 0, 255]  # 緑 (必要なら番号に応じて色を変える)
        combined_mask[mask > 0] = color  # 緑色で重ねる

    overlayed_image = cv2.addWeighted(image, 0.7, combined_mask, 0.3, 0)
    return overlayed_image

def generate_point_cloud(disp, image, mask):
    """深度マップと画像から点群を生成する関数"""
    H, W = disp.shape
    xx, yy = np.meshgrid(np.arange(W), np.arange(H))
    
    # 視差から深度を計算
    depth = (fx * baseline) / (-disp + cx2 - cx1)  # 深度マップの計算式
    
    # フライングポイントの除去
    mask[1:][np.abs(depth[1:] - depth[:-1]) > 1] = False
    mask[:, 1:][np.abs(depth[:, 1:] - depth[:, :-1]) > 1] = False

    # 3D点の計算
    points_grid = np.stack(((xx - cx1) / fx, (yy - cy1) / fy, np.ones_like(xx)), axis=0) * depth / 1000  # メートルに変換

    # マスク内の有効な点を抽出
    points = points_grid.transpose(1, 2, 0)[mask]
    colors = image[mask].astype(np.float64) / 255  # 色を[0,1]に正規化

    return points, colors

def save_merged_point_cloud(all_points, all_colors, output_path):
    """すべての点群と色をマージしてPLYファイルとして保存する関数"""
    # 点群と色をOpen3D形式に変換
    merged_point_cloud = o3d.geometry.PointCloud()

    # すべての点群と色を1つにまとめる
    merged_point_cloud.points = o3d.utility.Vector3dVector(np.vstack(all_points))
    merged_point_cloud.colors = o3d.utility.Vector3dVector(np.vstack(all_colors))

    # PLYとして保存
    o3d.io.write_point_cloud(str(output_path), merged_point_cloud)
    print(f"Merged point cloud saved to: {output_path}")

def process_image_with_yolo_and_depth(image_file, disp_file, model, output_dir):
    """YOLOでマスクを取得し、深度マップを使って点群を生成する関数"""
    image = load_image(image_file)
    disp = load_disp(disp_file)
    
    # モデルによる予測
    results = model.predict(source=image, conf=0.25, iou=0.45, imgsz=1120)
    ic(results)

    if results[0].masks is not None and len(results[0].masks) > 0:
        masks = results[0].masks.data.cpu().numpy()
        ic(masks.shape)

        all_points = []  # すべての点群を格納するリスト
        all_colors = []  # すべての色を格納するリスト

        # 各マスクについて点群を生成
        for i, mask in enumerate(masks):
            mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
            mask_bool = mask_resized > 0  # マスクをbool型に変換

            # 点群生成
            points, colors = generate_point_cloud(disp, image, mask_bool)

            # 点群と色をリストに追加
            all_points.append(points)
            all_colors.append(colors)

        # すべてのインスタンスの点群をマージして保存
        output_path = Path(output_dir) / "merged_point_cloud.ply"
        save_merged_point_cloud(all_points, all_colors, output_path)

    else:
        ic("No masks found in the results")


def main(image_pattern, disp_pattern, model_path, output_dir):
    model = YOLO(model_path)

    image_files = sorted(list(Path().glob(image_pattern)))
    disp_files = sorted(list(Path().glob(disp_pattern)))

    if len(image_files) != len(disp_files):
        print("Error: Mismatch in number of images and disparity files.")
        return

    for image_file, disp_file in zip(image_files, disp_files):
        print(f"Processing: {image_file} and {disp_file}")
        process_image_with_yolo_and_depth(image_file, disp_file, model, output_dir)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate point cloud from images and disparity maps using YOLO segmentation.")
    parser.add_argument("--images", required=True, help="Glob pattern for input image files.")
    parser.add_argument("--disparities", required=True, help="Glob pattern for input disparity files (.npy).")
    parser.add_argument("--model", required=True, help="Path to YOLO model file.")
    parser.add_argument("--output_dir", required=True, help="Directory to save point cloud PLY files.")
    
    args = parser.parse_args()

    main(args.images, args.disparities, args.model, args.output_dir)
