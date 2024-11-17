import cv2

# 入力画像のパス
image_path = "output_all_masks.png"  # 画像のパス

# 画像を読み込む
img = cv2.imread(image_path)

# 画像のサイズを表示
height, width, channels = img.shape
print(f"画像のサイズ: 高さ={height}, 幅={width}, チャンネル数={channels}")
