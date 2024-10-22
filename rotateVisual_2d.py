import numpy as np
import matplotlib.pyplot as plt


# Rotation matrix for rotation around the X axis
def rotation_matrix_x(theta):
    return np.array(
        [
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)],
        ]
    )


# Rotation matrix for rotation around the Y axis
def rotation_matrix_y(theta):
    return np.array(
        [
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)],
        ]
    )


# Rotation matrix for rotation around the Z axis
def rotation_matrix_z(theta):
    return np.array(
        [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ]
    )


# Function to plot the YZ plane with dashed lines after rotation
def plot_2d_plane_with_dashed_lines(
    ax, points_before, points_after, plane, label, color_before, color_after
):
    origin = np.array([0, 0])

    if plane == "yz":
        # Plot Y and Z axes before rotation
        ax.plot(
            [0, points_before[1][1]],
            [0, points_before[1][2]],
            color=color_before[1],
            label=f"{label} Y (Before)",
        )
        ax.plot(
            [0, points_before[2][1]],
            [0, points_before[2][2]],
            color=color_before[2],
            label=f"{label} Z (Before)",
        )
        # Plot Y and Z axes after rotation (dashed lines)
        ax.plot(
            [0, points_after[1][1]],
            [0, points_after[1][2]],
            color=color_after[1],
            linestyle="--",
            label=f"{label} Y (After)",
        )
        ax.plot(
            [0, points_after[2][1]],
            [0, points_after[2][2]],
            color=color_after[2],
            linestyle="--",
            label=f"{label} Z (After)",
        )

        ax.set_xlabel("Y axis")
        ax.set_ylabel("Z axis")
    elif plane == "xy":
        # Plot X and Y axes before rotation
        ax.plot(
            [0, points_before[0][0]],
            [0, points_before[0][1]],
            color=color_before[0],
            label=f"{label} X (Before)",
        )
        ax.plot(
            [0, points_before[1][0]],
            [0, points_before[1][1]],
            color=color_before[1],
            label=f"{label} Y (Before)",
        )
        # Plot X and Y axes after rotation (dashed lines)
        ax.plot(
            [0, points_after[0][0]],
            [0, points_after[0][1]],
            color=color_after[0],
            linestyle="--",
            label=f"{label} X (After)",
        )
        ax.plot(
            [0, points_after[1][0]],
            [0, points_after[1][1]],
            color=color_after[1],
            linestyle="--",
            label=f"{label} Y (After)",
        )
        ax.set_xlabel("X axis")
        ax.set_ylabel("Y axis")
    elif plane == "xz":
        # Plot X and Z axes before rotation
        ax.plot(
            [0, points_before[0][0]],
            [0, points_before[0][2]],
            color=color_before[0],
            label=f"{label} X (Before)",
        )
        ax.plot(
            [0, points_before[2][0]],
            [0, points_before[2][2]],
            color=color_before[2],
            label=f"{label} Z (Before)",
        )
        # Plot X and Z axes after rotation (dashed lines)
        ax.plot(
            [0, points_after[0][0]],
            [0, points_after[0][2]],
            color=color_after[0],
            linestyle="--",
            label=f"{label} X (After)",
        )
        ax.plot(
            [0, points_after[2][0]],
            [0, points_after[2][2]],
            color=color_after[2],
            linestyle="--",
            label=f"{label} Z (After)",
        )
        ax.set_xlabel("X axis")
        ax.set_ylabel("Z axis")

    ax.legend()
    ax.grid(True)
    ax.set_title(f"{label} in {plane.upper()} plane")


# Function to extract transformed axes points
def extract_axes_points(matrix):
    x_axis = np.dot(matrix, np.array([1, 0, 0]))
    y_axis = np.dot(matrix, np.array([0, 1, 0]))
    z_axis = np.dot(matrix, np.array([0, 0, 1]))
    return [x_axis, y_axis, z_axis]


if __name__ == "__main__":

    # Define initial identity matrix for comparison
    identity_matrix = np.eye(3)

    # Rotation angle for pitch (30 degrees, converted to radians)
    theta_x = np.radians(30)  # For X-axis (pitch)
    theta_y = np.radians(30)  # For Y-axis (roll)
    theta_z = np.radians(30)  # For Z-axis (yaw)

    # Get rotation matrix for X-axis
    rotation_x_corrected = rotation_matrix_x(theta_x)
    rotation_y_corrected = rotation_matrix_y(theta_y)
    rotation_z_corrected = rotation_matrix_z(theta_z)

    # Extract points before and after rotation
    initial_axes_points = extract_axes_points(identity_matrix)
    points_step_x = extract_axes_points(rotation_x_corrected)
    # Extract points before and after Y-axis rotation
    points_step_y = extract_axes_points(rotation_y_corrected)
    # Extract points before and after Z-axis rotation
    points_step_z = extract_axes_points(rotation_z_corrected)

    # Plot the YZ plane before and after rotation
    fig, axs = plt.subplots(1, 3, figsize=(12, 6))

    # Step 1: Rotate around X axis (YZ plane)
    plot_2d_plane_with_dashed_lines(
        axs[0],
        initial_axes_points,
        points_step_x,
        "yz",
        "Step 1 (pitch)",
        ["r", "g", "b"],
        ["m", "y", "c"],
    )
    # Step 2: Rotate around Y axis (XZ plane)
    plot_2d_plane_with_dashed_lines(
        axs[1],
        initial_axes_points,
        points_step_y,
        "xz",
        "Step 2 (Yaw)",
        ["r", "g", "b"],
        ["m", "y", "c"],
    )
    # Step 3: Rotate around Z axis (XY plane)
    plot_2d_plane_with_dashed_lines(
        axs[2],
        initial_axes_points,
        points_step_z,
        "xy",
        "Step 3 (Roll)",
        ["r", "g", "b"],
        ["m", "y", "c"],
    )

    plt.tight_layout()
    plt.savefig("2d-explain.png", dpi=300, bbox_inches="tight", format="png")
    # plt.show()
