import numpy as np
import csv
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import colorsys
import sys

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
    rotation = R.from_quat([rx, ry, rz, rw])
    return translation, rotation

def transform_points(points, translation, rotation):
    transformed_points = rotation.apply(points) + translation
    return transformed_points

def rgb_to_hsv(color):
    """Convert an RGB color to HSV format."""
    r, g, b = color / 255.0  # Normalize RGB to [0, 1]
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    return np.array([h, s, v])

def filter_points(points, colors):
    # Convert RGB to HSV for color filtering
    hsv_colors = np.array([rgb_to_hsv(color) for color in colors])
    
    # Define green and white ranges in HSV
    green_lower_bound = np.array([0.25 - 0.15, 0.2, 0.2])
    green_upper_bound = np.array([0.25 + 0.15, 1.0, 1.0])
    white_lower_bound = np.array([0, 0.0, 0.4])
    white_upper_bound = np.array([1.0, 0.5, 1.0])
    
    # Create masks for filtering green and white colors
    green_mask = np.all((hsv_colors >= green_lower_bound) & (hsv_colors <= green_upper_bound), axis=1)
    white_mask = np.all((hsv_colors >= white_lower_bound) & (hsv_colors <= white_upper_bound), axis=1)
    
    # Exclude points that are green or white
    mask = ~(green_mask | white_mask)
    return points[mask], colors[mask]

def save_transformed_ply(ply_path, points, colors):
    output_path = ply_path.parent / (ply_path.stem + "_transformed.ply")
    with open(output_path, 'w') as f:
        f.write("ply\nformat ascii 1.0\nelement vertex {}\n".format(len(points)))
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for point, color in zip(points, colors):
            f.write("{} {} {} {} {} {}\n".format(*point, *color))
    print(f"Saved transformed PLY to {output_path}")

# Main function to apply transformation and filtering to PLY points
def main(ply_folder_or_file):
    # Check if the argument is a file or a folder
    ply_path = Path(ply_folder_or_file)
    
    if ply_path.is_file() and ply_path.suffix == '.ply':
        # Process a single PLY file
        csv_path = ply_path.with_name("tf" + ply_path.stem + ".csv")
        if not csv_path.exists():
            print(f"CSV file for {ply_path} not found.")
            return
        
        # Load points and colors
        points, colors = load_ply(ply_path)
        
        # Apply transformation
        translation, rotation = load_csv_transform(csv_path)
        transformed_points = transform_points(points, translation, rotation)
        
        # Apply color filters
        filtered_points, filtered_colors = filter_points(transformed_points, colors)
        
        # Save filtered and transformed PLY
        save_transformed_ply(ply_path, transformed_points, colors)

    elif ply_path.is_dir():
        # Process all PLY files in the given directory
        for ply_path in ply_path.glob("*/**/*.ply"):
            csv_path = ply_path.with_name("tf" + ply_path.stem + ".csv")
            if not csv_path.exists():
                print(f"CSV file for {ply_path} not found.")
                continue
            
            # Load points and colors
            points, colors = load_ply(ply_path)
            
            # Apply transformation
            translation, rotation = load_csv_transform(csv_path)
            transformed_points = transform_points(points, translation, rotation)
            
            # Apply color filters
            filtered_points, filtered_colors = filter_points(transformed_points, colors)
            
            # Save filtered and transformed PLY
            save_transformed_ply(ply_path, filtered_points, filtered_colors)

# Check if the script is run directly
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <path_to_ply_folder_or_file>")
    else:
        main(sys.argv[1])
