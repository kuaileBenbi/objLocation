import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def plot_coordinate_system(ax, R, title):
    # Original coordinate system (solid lines)
    ax.quiver(0, 0, 0, 1, 0, 0, color="r", linestyle="-", label="X axis (before)")
    ax.quiver(0, 0, 0, 0, 1, 0, color="g", linestyle="-", label="Y axis (before)")
    ax.quiver(0, 0, 0, 0, 0, 1, color="b", linestyle="-", label="Z axis (before)")

    # Rotated coordinate system (dashed lines)
    rotated_x = R @ np.array([1, 0, 0])
    rotated_y = R @ np.array([0, 1, 0])
    rotated_z = R @ np.array([0, 0, 1])

    ax.quiver(
        0,
        0,
        0,
        rotated_x[0],
        rotated_x[1],
        rotated_x[2],
        color="r",
        linestyle="--",
        label="X axis (after)",
    )
    ax.quiver(
        0,
        0,
        0,
        rotated_y[0],
        rotated_y[1],
        rotated_y[2],
        color="g",
        linestyle="--",
        label="Y axis (after)",
    )
    ax.quiver(
        0,
        0,
        0,
        rotated_z[0],
        rotated_z[1],
        rotated_z[2],
        color="b",
        linestyle="--",
        label="Z axis (after)",
    )

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()


# Define rotation matrices
def rotation_matrix(axis, angle):
    angle = np.radians(angle)
    if axis == "X":
        return np.array(
            [
                [1, 0, 0],
                [0, np.cos(angle), np.sin(angle)],
                [0, -np.sin(angle), np.cos(angle)],
            ]
        )
    elif axis == "Y":
        return np.array(
            [
                [np.cos(angle), 0, -np.sin(angle)],
                [0, 1, 0],
                [np.sin(angle), 0, np.cos(angle)],
            ]
        )
    elif axis == "Z":
        return np.array(
            [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ]
        )


# Given angles for camera and platform rotations
# platform_yaw, platform_pitch, platform_roll = 246.54, -0.22, 2.09
camera_yaw, camera_pitch = 36.88, 1.82

platform_yaw, platform_pitch, platform_roll = -113.46, -0.22, 2.09
# camera_yaw, camera_pitch = 283.42, 1.60

# platform_yaw, platform_pitch, platform_roll = 60, 45, 30
# platform_yaw, platform_pitch, platform_roll = 0, 0, 0
# camera_yaw, camera_pitch = 0, 0
# camera_yaw, camera_pitch = 10, 10

# Compute individual rotation matrices
R1 = rotation_matrix("X", camera_pitch)  # Step 1: Pitch around XC axis
R2 = rotation_matrix("Z", camera_yaw)  # Step 2: Yaw around ZC axis
R3 = rotation_matrix("Y", platform_roll)  # Step 3: Roll around YC axis
R4 = rotation_matrix("X", platform_pitch)  # Step 4: Pitch around XC axis
R5 = rotation_matrix("Z", platform_yaw)  # Step 5: Yaw around ZC axis

# Compute the final rotation matrix (product of all matrices)
R_final = R1 @ R2 @ R3 @ R4 @ R5
alpha = np.arctan2(R_final[1, 0], R_final[1, 1])  # yaw -180~+180
beta = np.arcsin(R_final[1, 2])  # pitch -90~+90
gamma = -np.arctan(R_final[0, 2] / R_final[2, 2])  # roll -90~+90
print(f"R_final: {R_final}")
print(f"yaw:{np.degrees(alpha)}, pitch:{np.degrees(beta)}, roll:{np.degrees(gamma)}")

R_final_ = R_final.T
print(f"R_final_: {R_final_}")
alpha = np.arctan2(R_final_[0, 1], R_final_[1, 1])  # yaw
beta = np.arcsin(R_final_[2, 1])  # pitch
gamma = -np.arctan(R_final_[2, 0] / R_final_[2, 2])  # roll
print(f"yaw:{np.degrees(alpha)}, pitch:{np.degrees(beta)}, roll:{np.degrees(gamma)}")

"""
fig = plt.figure(figsize=(12, 10))

ax1 = fig.add_subplot(231, projection="3d")
plot_coordinate_system(ax1, R1, "Step 1: Pitch (X-axis)")

ax2 = fig.add_subplot(232, projection="3d")
R12 = R2 @ R1  # Combine rotations
plot_coordinate_system(ax2, R12, "Step 2: Yaw (Z-axis)")

ax3 = fig.add_subplot(233, projection="3d")
R123 = R3 @ R12  # Combine rotations
plot_coordinate_system(ax3, R123, "Step 3: Roll (Y-axis)")

ax4 = fig.add_subplot(234, projection="3d")
R1234 = R4 @ R123  # Combine rotations
plot_coordinate_system(ax4, R1234, "Step 4: Pitch (X-axis)")

ax5 = fig.add_subplot(235, projection="3d")
R12345 = R5 @ R1234  # Combine rotations
plot_coordinate_system(ax5, R12345, "Step 5: Yaw (Z-axis)")

plt.tight_layout()
# plt.show()
plt.savefig("3d-explains.png", dpi=300, bbox_inches="tight", format="png")
"""


def plot_coordinate_system(ax, origin, R, label_prefix, color, scale=0.2):
    """Plot the coordinate system axes defined by the rotation matrix R."""
    # Plot the X axis
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        R[0, 0] * scale,
        R[1, 0] * scale,
        R[2, 0] * scale,
        color=color,
        linestyle="-",
        label=f"{label_prefix} X",
    )
    # Plot the Y axis
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        R[0, 1] * scale,
        R[1, 1] * scale,
        R[2, 1] * scale,
        color=color,
        linestyle="-",
        label=f"{label_prefix} Y",
    )
    # Plot the Z axis
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        R[0, 2] * scale,
        R[1, 2] * scale,
        R[2, 2] * scale,
        color=color,
        linestyle="-",
        label=f"{label_prefix} Z",
    )


identity_matrix = np.eye(3)
optical_axis_initial = np.array([0, 1, 0])
optical_axis_final = R_final @ optical_axis_initial

print(f"optical_axis_final: {optical_axis_final}")

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection="3d")

# Plot initial coordinate system (no rotation, identity matrix)
plot_coordinate_system(ax, [0, 0, 0], identity_matrix, "Initial", "b")

# Plot final coordinate system after all rotations
# plot_coordinate_system(ax, [0, 0, 0], R_final, "Final", "r")

# Plot the initial optical axis (solid line, initial position)
ax.quiver(
    0,
    0,
    0,
    optical_axis_initial[0],
    optical_axis_initial[1],
    optical_axis_initial[2],
    color="g",
    linestyle="-",
    label="Initial Optical Axis",
    linewidth=2,
)

# Plot final optical axis (dashed line, final position after all rotations)
ax.quiver(
    0,
    0,
    0,
    optical_axis_final[0],
    optical_axis_final[1],
    optical_axis_final[2],
    color="r",
    linestyle="--",
    label="Final Optical Axis",
    linewidth=2,
)

# Set plot limits and labels
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title("3D Visualization of Camera Optical Axis and Coordinate Systems")

# Add legend
ax.legend()
plt.savefig("3d-explains.png", dpi=300, bbox_inches="tight", format="png")
