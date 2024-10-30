import argparse

def convert_to_meters(input_ply, output_ply):
    with open(input_ply, 'r') as f:
        lines = f.readlines()
    
    # ヘッダーを処理
    header = []
    vertex_start = 0
    for i, line in enumerate(lines):
        header.append(line)
        if line.startswith("end_header"):
            vertex_start = i + 1
            break
    
    # 座標を1/1000倍してメートルに変換
    converted_lines = header[:]
    for line in lines[vertex_start:]:
        parts = line.split()
        
        # 座標変換 (最初の3つの要素が x, y, z の座標と仮定)
        x, y, z = map(float, parts[:3])
        x /= 1000.0
        y /= 1000.0
        z /= 1000.0
        
        # カラー情報がある場合もそのまま追加
        if len(parts) > 3:
            color = parts[3:]
            new_line = f"{x:.6f} {y:.6f} {z:.6f} " + " ".join(color) + "\n"
        else:
            new_line = f"{x:.6f} {y:.6f} {z:.6f}\n"
        
        converted_lines.append(new_line)
    
    # 出力ファイルに保存
    with open(output_ply, 'w') as f:
        f.writelines(converted_lines)
    
    print(f"Converted PLY file saved to: {output_ply}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert PLY file coordinates to meters.")
    parser.add_argument("input_ply", type=str, help="Path to the input PLY file")
    parser.add_argument("output_ply", type=str, help="Path to the output PLY file")
    args = parser.parse_args()
    
    convert_to_meters(args.input_ply, args.output_ply)
