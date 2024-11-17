import argparse
import numpy as np
from pathlib import Path
from imageio.v3 import imread
import struct
from ultralytics import YOLO

# Calibration parameters
fx, fy, cx1, cy = 1400.6, 1400.6, 1103.65, 574.575
cx2 = 1102.84
baseline = 62.8749  # in millimeters
Rotation_Z_90 = np.array([
    [0, 1, 0],
    [-1, 0, 0],
    [0, 0, 1]
])

def write_ply(filename, points, colors):
    header = f'''ply
format binary_little_endian 1.0
element vertex {len(points)}
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''
    with open(filename, 'wb') as f:
        f.write(header.encode('utf-8'))
        for point, color in zip(points, colors):
            f.write(struct.pack('<fffBBB',
                                point[0], point[1], point[2],
                                int(color[0] * 255), int(color[1] * 255), int(color[2] * 255)))

def process_image_with_yolo_and_depth(image_path, disp_path, model):
    # Load the image
    image = imread(image_path)
    
    # Run YOLO segmentation
    results = model(image)
    # Combine all masks into a single binary mask
    masks = results[0].masks.cpu().numpy().masks  # All masks in the result
    combined_mask = np.any(masks, axis=0)  # Logical OR operation across all masks

    # Load the disparity file
    disp = np.load(disp_path)
    
    # Compute depth
    depth = (fx * baseline) / (-disp + (cx2 - cx1))
    H, W = depth.shape
    xx, yy = np.meshgrid(np.arange(W), np.arange(H))

    # Mask 2D pixels
    mask = combined_mask.astype(bool)
    valid_xx = xx[mask]
    valid_yy = yy[mask]
    valid_depth = depth[mask]
    valid_colors = image[mask].astype(np.float64) / 255.0

    # Generate 3D points
    valid_points = np.stack([
        (valid_xx - cx1) / fx * valid_depth,
        (valid_yy - cy) / fy * valid_depth,
        valid_depth
    ], axis=1) / 1000.0  # Convert to meters

    # Rotate points 90 degrees around Z-axis
    rotated_points = np.dot(valid_points, Rotation_Z_90.T)

    # Save the points and colors to a PLY file
    output_dir = Path(image_path).parent
    ply_filename = output_dir / f"{Path(image_path).stem}_segmented.ply"
    write_ply(ply_filename, rotated_points, valid_colors)
    print(f"PLY file saved: {ply_filename}")

def main(image_pattern, disp_pattern, model_path):
    # Load YOLO model
    model = YOLO(model_path)

    # Get sorted file lists
    image_files = sorted(list(Path().glob(image_pattern)))
    disp_files = sorted(list(Path().glob(disp_pattern)))

    if len(image_files) != len(disp_files):
        print("Error: Mismatch in number of images and disparity files.")
        return

    for image_file, disp_file in zip(image_files, disp_files):
        print(f"Processing: {image_file} and {disp_file}")
        process_image_with_yolo_and_depth(image_file, disp_file, model)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate point cloud from images and disparity maps using YOLO segmentation.")
    parser.add_argument("--images", required=True, help="Glob pattern for input image files.")
    parser.add_argument("--disparities", required=True, help="Glob pattern for input disparity files (.npy).")
    parser.add_argument("--model", required=True, help="Path to YOLO model file.")
    args = parser.parse_args()

    main(args.images, args.disparities, args.model)
