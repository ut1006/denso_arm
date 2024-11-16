import cv2
import numpy as np
from pathlib import Path
from ultralytics import YOLO

# YOLOモデルのロード 
model_path = "/home/kamadagpu/catkin_ws/src/denso_arm/scripts/my_moveit/YOLO/roboflow/weight/best.pt"
model = YOLO(model_path)  # モデルをロード
# 入力画像の読み込み
image_path = "l0014.png"  # 画像のパス
img = cv2.imread(image_path)

# YOLOで画像のインスタンスセグメンテーションを実行
results = model(img)

# 画像に検出結果を描画
for r in results:
    # 各オブジェクトのインスタンスマスクを処理
    for i, c in enumerate(r):
        # マスクを取得
        mask = c.masks  # インスタンスセグメンテーションのマスク

        # 各オブジェクトのマスクを描画
        if mask is not None:
            # CUDAからCPUに移動してからNumPy配列に変換
            binary_mask = mask.cpu().numpy()  # マスクをCPUに移動
            binary_mask = (binary_mask * 255).astype(np.uint8)  # 0-255のバイナリ形式に変換

            # バイナリマスクを保存（オプション）
            mask_output_path = f"mask_{Path(image_path).stem}_{i}.png"
            cv2.imwrite(mask_output_path, binary_mask)

            # バイナリマスクをオーバーレイ（表示または画像に描画）
            img_with_mask = cv2.bitwise_and(img, img, mask=binary_mask)
            cv2.imshow(f"Mask for Object {i}", img_with_mask)

# 最後に結果の画像を表示（オプション）
cv2.imshow("Original Image with Masks", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
