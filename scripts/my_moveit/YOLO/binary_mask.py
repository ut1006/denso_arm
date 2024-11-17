import cv2  # OpenCV をインポート
from imageio.v3 import imread
import numpy as np
from pathlib import Path
from ultralytics import YOLO
from icecream import ic
# python binary_mask.py --images l0014.png --disparities 0014.npy --model /home/kamadagpu/catkin_ws/src/denso_arm/scripts/my_moveit/YOLO/roboflow/weight/best.pt 

# icecream を無効化
ic.disable()

def load_image(image_file):
    """画像を読み込む関数"""
    image = imread(image_file)  # imageio を使って画像を読み込む
    ic(image.shape)  # 画像の形状を確認
    return image

def load_depth(disp_file):
    """深度マップを読み込む関数"""
    depth = np.load(disp_file)  # .npy ファイルから深度をロード
    ic(depth.shape)  # 深度マップの形状を確認
    return depth

def overlay_mask(image, mask):
    """画像にマスクを重ねる関数"""
    # マスクをリサイズ (必要に応じて)
    mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)

    # マスクをカラー形式に変換
    mask_colored = np.zeros_like(image)
    mask_colored[mask > 0] = [0, 255, 0]  # 緑色でマスクを塗る

    # 画像とマスクを重ねる
    overlayed_image = cv2.addWeighted(image, 0.7, mask_colored, 0.3, 0)
    return overlayed_image

def overlay_all_masks(image, masks):
    """すべてのマスクを1つの画像に重ねる"""
    combined_mask = np.zeros_like(image)
    for i, mask in enumerate(masks):
        # マスクをリサイズ
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
        
        # マスクごとに色を変更 (例: マスクの番号で色を変える)
        color = [0, 0, 255]  # 緑 (必要なら番号に応じて色を変える)
        combined_mask[mask > 0] = color  # 緑色で重ねる

    # 元の画像とマスクを重ねる
    overlayed_image = cv2.addWeighted(image, 0.7, combined_mask, 0.3, 0)
    return overlayed_image

def process_image_with_yolo_and_depth(image_file, disp_file, model):
    ic(image_file, disp_file)

    # 画像と深度情報を読み込む
    image = load_image(image_file)
    depth = load_depth(disp_file)
    ic(image.shape, depth.shape)

    # モデルによる予測
    results = model.predict(source=image, conf=0.25, iou=0.45, imgsz=1120)
    ic(results)

    if results[0].masks is not None:
        try:
            masks = results[0].masks.data.cpu().numpy()
            ic(masks.shape)

            # すべてのマスクを重ねる
            overlayed_image = overlay_all_masks(image, masks)
            output_file = "output_all_masks.png"
            cv2.imwrite(output_file, overlayed_image)
            print(f"Combined mask overlay saved to: {output_file}")
        except Exception as e:
            ic(e)
    else:
        ic("No masks found in the results")

#各セグメントごとに画像保存
# def process_image_with_yolo_and_depth(image_file, disp_file, model):
#     ic(image_file, disp_file)

#     # 画像と深度情報を読み込む
#     image = load_image(image_file)
#     depth = load_depth(disp_file)
#     ic(image.shape, depth.shape)

#     # モデルによる予測
#     results = model.predict(image)
#     ic(results)

#     if results[0].masks is not None:
#         try:
#             masks = results[0].masks.data.cpu().numpy()
#             ic(masks.shape)

#             # 最初のマスクを画像に重ねる (例として1つ目のマスクを使用)
#             for i, mask in enumerate(masks):
#                 overlayed_image = overlay_mask(image, mask)
#                 output_file = f"output_mask_{i}.png"
#                 cv2.imwrite(output_file, overlayed_image)
#                 print(f"Mask overlay saved to: {output_file}")
#         except Exception as e:
#             ic(e)
#     else:
#         ic("No masks found in the results")

def main(image_pattern, disp_pattern, model_path):
    model = YOLO(model_path)

    image_files = sorted(list(Path().glob(image_pattern)))
    disp_files = sorted(list(Path().glob(disp_pattern)))

    if len(image_files) != len(disp_files):
        print("Error: Mismatch in number of images and disparity files.")
        return

    for image_file, disp_file in zip(image_files, disp_files):
        print(f"Processing: {image_file} and {disp_file}")
        process_image_with_yolo_and_depth(image_file, disp_file, model)
def main(image_pattern, disp_pattern, model_path):
    model = YOLO(model_path)

    image_files = sorted(list(Path().glob(image_pattern)))
    disp_files = sorted(list(Path().glob(disp_pattern)))

    if len(image_files) != len(disp_files):
        print("Error: Mismatch in number of images and disparity files.")
        return

    for image_file, disp_file in zip(image_files, disp_files):
        print(f"Processing: {image_file} and {disp_file}")
        process_image_with_yolo_and_depth(image_file, disp_file, model)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate point cloud from images and disparity maps using YOLO segmentation.")
    parser.add_argument("--images", required=True, help="Glob pattern for input image files.")
    parser.add_argument("--disparities", required=True, help="Glob pattern for input disparity files (.npy).")
    parser.add_argument("--model", required=True, help="Path to YOLO model file.")
    args = parser.parse_args()

    main(args.images, args.disparities, args.model)
