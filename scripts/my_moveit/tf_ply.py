import numpy as np
import csv
from pathlib import Path
from scipy.spatial.transform import Rotation as R

def load_ply(ply_path):
    points, colors = [], []
    with open(ply_path, 'r') as f:
        data = f.readlines()
        vertex_section = False
        for line in data:
            if line.startswith("end_header"):
                vertex_section = True
            elif vertex_section:
                x, y, z, r, g, b = map(float, line.split())
                points.append([x, y, z])
                colors.append([int(r), int(g), int(b)])
    return np.array(points), np.array(colors)

def load_csv_transform(csv_path):
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        row = next(reader)
        tx, ty, tz = map(float, row[:3])
        rx, ry, rz, rw = map(float, row[3:])
    translation = np.array([tx, ty, tz])
    rotation = R.from_quat([rx, ry, rz, rw]).inv()
    return translation, rotation

def transform_points(points, translation, rotation):
    # Apply inverse rotation and translation
    transformed_points = rotation.apply(points) - translation
    return transformed_points

def save_transformed_ply(ply_path, points, colors):
    # Create a new filename with "_transformed" suffix
    output_path = ply_path.parent / (ply_path.stem + "_transformed.ply")
    
    with open(output_path, 'w') as f:
        f.write("ply\nformat ascii 1.0\nelement vertex {}\n".format(len(points)))
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for point, color in zip(points, colors):
            f.write("{} {} {} {} {} {}\n".format(*point, *color))
    print(f"Saved transformed PLY to {output_path}")

# Main function to apply transformation to PLY points
def main(ply_folder):
    for ply_path in Path(ply_folder).glob("*/**/*.ply"):
        csv_path = ply_path.with_name("tf" + ply_path.stem + ".csv")
        if not csv_path.exists():
            print(f"CSV file for {ply_path} not found.")
            continue
        points, colors = load_ply(ply_path)
        translation, rotation = load_csv_transform(csv_path)
        transformed_points = transform_points(points, translation, rotation)
        save_transformed_ply(ply_path, transformed_points, colors)

# Run the transformation
main("output")
