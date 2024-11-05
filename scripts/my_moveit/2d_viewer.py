image_path = "output/0013/l0013.png"
import tkinter as tk
from PIL import Image, ImageTk
import colorsys

# 画像を読み込む
image = Image.open(image_path).convert("RGB")

# HSVに変換した画像を用意
hsv_image = image.convert("HSV")

# グローバル変数でドラッグ開始位置と終了位置を保持
start_x, start_y = None, None
end_x, end_y = None, None

def on_mouse_down(event):
    global start_x, start_y
    start_x, start_y = event.x, event.y  # ドラッグ開始位置を保存

def on_mouse_drag(event):
    global end_x, end_y
    end_x, end_y = event.x, event.y  # ドラッグ終了位置を更新
    # 矩形を描画
    canvas.delete("rectangle")
    canvas.create_rectangle(start_x, start_y, end_x, end_y, outline="red", tag="rectangle")

# グローバル変数で総計の最大・最小HSVを保持
all_max_hsv = (0, 0, 0)
all_min_hsv = (360, 255, 255)

def on_mouse_up(event):
    global start_x, start_y, end_x, end_y, all_max_hsv, all_min_hsv
    end_x, end_y = event.x, event.y
    
    # 範囲が画像の中に収まるようにクリッピング
    start_x_clip = max(0, min(start_x, image.width - 1))
    start_y_clip = max(0, min(start_y, image.height - 1))
    end_x_clip = max(0, min(end_x, image.width - 1))
    end_y_clip = max(0, min(end_y, image.height - 1))
    
    # 選択範囲を取得
    left, top = min(start_x_clip, end_x_clip), min(start_y_clip, end_y_clip)
    right, bottom = max(start_x_clip, end_x_clip), max(start_y_clip, end_y_clip)
    
    # HSVの最大・最小値を計算
    if left < right and top < bottom:
        selected_region = hsv_image.crop((left, top, right, bottom))
        hsv_values = list(selected_region.getdata())
        h_values = [h for h, _, _ in hsv_values]
        s_values = [s for _, s, _ in hsv_values]
        v_values = [v for _, _, v in hsv_values]
        
        # 各チャンネルの範囲を計算
        max_hsv = (max(h_values) / 360, max(s_values) / 255, max(v_values) / 255)
        min_hsv = (min(h_values) / 360, min(s_values) / 255, min(v_values) / 255)
        
        # Hだけ分数表記、SとVは0〜1で出力
        max_hsv_output = (f"{int(max(h_values))}/360", round(max_hsv[1], 3), round(max_hsv[2], 3))
        min_hsv_output = (f"{int(min(h_values))}/360", round(min_hsv[1], 3), round(min_hsv[2], 3))
        
        print(f"Selected region HSV max: {max_hsv_output}, min: {min_hsv_output}")
        
        # 全体の最大・最小を更新
        all_max_hsv = (
            max(all_max_hsv[0], max_hsv[0]),
            max(all_max_hsv[1], max_hsv[1]),
            max(all_max_hsv[2], max_hsv[2])
        )
        all_min_hsv = (
            min(all_min_hsv[0], min_hsv[0]),
            min(all_min_hsv[1], min_hsv[1]),
            min(all_min_hsv[2], min_hsv[2])
        )

# ウィンドウ終了時にトータルの最大・最小を出力
def on_closing():
    final_max_hsv = (
        f"{int(all_max_hsv[0] * 360)}/360", round(all_max_hsv[1], 3), round(all_max_hsv[2], 3)
    )
    final_min_hsv = (
        f"{int(all_min_hsv[0] * 360)}/360", round(all_min_hsv[1], 3), round(all_min_hsv[2], 3)
    )
    print(f"Overall normalized HSV max: {final_max_hsv}, min: {final_min_hsv}")
    root.destroy()

# Tkinterウィンドウ設定とイベントバインド
root = tk.Tk()
root.title("Image Viewer with HSV Range on Selection")

# 画像をTkinter用に変換して表示
photo = ImageTk.PhotoImage(image)
canvas = tk.Canvas(root, width=image.width, height=image.height)
canvas.pack()
canvas.create_image(0, 0, anchor=tk.NW, image=photo)

# マウスイベントをバインド
canvas.bind("<Button-1>", on_mouse_down)
canvas.bind("<B1-Motion>", on_mouse_drag)
canvas.bind("<ButtonRelease-1>", on_mouse_up)

# ウィンドウ閉じる時の処理をバインド
root.protocol("WM_DELETE_WINDOW", on_closing)

root.mainloop()
