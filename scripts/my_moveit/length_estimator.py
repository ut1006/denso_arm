import numpy as np
import matplotlib.pyplot as plt
from skimage.io import imread
from pathlib import Path
import open3d as o3d
import cv2

# Camera parameters (adjust these values based on your calibration)
fx, fy, cx1, cy = 1400.6, 1400.6, 1103.65, 574.575
cx2 = 1102.84
baseline = 62.8749  # in millimeters

def load_and_generate_point_cloud(disp_path):
    # Load disparity
    disp = np.load(disp_path)

    # Inverse project
    depth = (fx * baseline) / (-disp + (cx2 - cx1))/1000
    return depth, disp.shape

def on_mouse_click(event, x, y, flags, param):
    selected_points, depth, img_shape, img = param
    if event == cv2.EVENT_LBUTTONDOWN:
        # Ensure the coordinates are within bounds
        if x >= 0 and x < img_shape[1] and y >= 0 and y < img_shape[0]:
            # Calculate the corresponding depth value
            Z = depth[img_shape[0] - 1 - y, x]  # Invert y for depth lookup
            if Z > 0:  # Ensure Z is valid
                # Convert pixel coordinates to 3D coordinates
                X = (x - cx1) * Z / fx
                Y = (img_shape[0] - 1 - y - cy) * Z / fy  # Invert y for conversion
                selected_points.append(np.array([X, Y, Z]))

                # Draw the point on the image
                cv2.circle(img, (x, y), 5, (0, 255, 0), -1)  # Green circle
                cv2.imshow('img', img)

                # If two points are selected, calculate the distance
                if len(selected_points) == 2:
                    distance = np.linalg.norm(selected_points[0] - selected_points[1])
                    print(f"Distance between selected 3D points: {distance:.3f} meters")
                    # Optionally, you can keep the window open or close it here
                    # Uncomment the next line if you want to close it after measuring
                    # cv2.destroyAllWindows()  # Close the image window

def visualize_and_measure_distance(depth, img):
    selected_points = []

    cv2.imshow('img', img)
    cv2.setMouseCallback("img", on_mouse_click, param=(selected_points, depth, img.shape, img))

    # Wait until the user decides to close the image window
    while True:
        key = cv2.waitKey(1)
        if key == 27:  # Esc key to close
            break

    cv2.destroyAllWindows()

def main():
    # Define paths for your disparity map
    disp_path = 'output/0013/0013.npy'  # Example path for the disparity map

    # Load depth data
    depth, img_shape = load_and_generate_point_cloud(disp_path)

    # Load the original image for display
    img = imread('output/0013/l0013.png')

    # Visualize and measure distance between selected points
    visualize_and_measure_distance(depth, img)

if __name__ == "__main__":
    main()
