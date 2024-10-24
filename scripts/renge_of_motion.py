import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Joint lengths
lengths = [0.1815, 0.1635, 0.3050, 0.1645, 0.1355, 0.0700]

# Joint limits in radians
joint_limits = [
    (-2.9, 2.9),   # joint_1
    (-1.5, 1.5),   # joint_2
    (-0.5, 2.7),   # joint_3
    (-2.0, 2.0),   # joint_4
    (-1.5, 1.5),   # joint_5
    (-3.1, 3.1)    # joint_6
]

# Define the function to calculate joint positions
def calculate_positions(lengths, angles):
    n = len(lengths)  # Number of joints
    positions = np.zeros((n + 1, 3))  # Initialize positions (n joints + base)

    # Calculate positions iteratively in 3D
    for i in range(n):
        positions[i + 1, 0] = positions[i, 0] + lengths[i] * np.cos(angles[i])  # X coordinate
        positions[i + 1, 1] = positions[i, 1] + lengths[i] * np.sin(angles[i])  # Y coordinate
        positions[i + 1, 2] = positions[i, 2]  # Keep Z constant or adjust if needed

    return positions

# Generate angle combinations considering joint limits
angles = [np.linspace(limit[0], limit[1], num=10) for limit in joint_limits]
all_positions = []

# Generate all combinations
for angle_combination in np.meshgrid(*angles):
    combined_angles = [a.flatten() for a in angle_combination]
    combined_angles = np.array(combined_angles).flatten()  # Flatten to get a single array
    positions = calculate_positions(lengths, combined_angles)  # Pass both lengths and angles
    all_positions.append(positions)

# Visualization
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Draw the arm for each angle combination
for positions in all_positions:
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], alpha=0.1, color='b')

# Set axes
ax.set_title('Robot Arm Visualization with Joint Limits')
ax.set_xlabel('X position (m)')
ax.set_ylabel('Y position (m)')
ax.set_zlabel('Z position (m)')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([0, 1])
ax.grid()

plt.show()
