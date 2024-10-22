import numpy as np


class TargetLocator:
    # 地球椭球模型常量（WGS84）
    EARTH_EQUATORIAL_RADIUS = 6378137.0  # 地球赤道半径，单位：米
    EARTH_POLAR_RADIUS = 6356752.3  # 地球极半径，单位：米
    FLATTENING_FACTOR = 1 / 298.257223563  # 扁率
    FIRST_ECCENTRICITY_SQUARED = FLATTENING_FACTOR * (
        2 - FLATTENING_FACTOR
    )  # 第一偏心率平方

    def __init__(self, K):
        # Initialize with camera intrinsic parameters
        self.K = K

    @staticmethod
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

    def compute_attitude_angles(
        self,
        platform_yaw,
        platform_pitch,
        platform_roll,
        camera_yaw,
        camera_pitch,
        mirror_pitch=None,
        mirror_roll=None,
    ):
        # Calculate the final rotation matrix
        R1 = self.rotation_matrix("X", camera_pitch)
        R2 = self.rotation_matrix("Z", camera_yaw)
        R3 = self.rotation_matrix("Y", platform_roll)
        R4 = self.rotation_matrix("X", platform_pitch)
        R5 = self.rotation_matrix("Z", platform_yaw)

        if mirror_pitch is not None and mirror_roll is not None:
            R_mirror_pitch = self.rotation_matrix("X", mirror_pitch)  # 快摆镜的俯仰角
            R_mirror_roll = self.rotation_matrix("Y", mirror_roll)  # 快摆镜的横滚角
            R_final = R_mirror_roll @ R_mirror_pitch @ R1 @ R2 @ R3 @ R4 @ R5
        else:
            R_final = R1 @ R2 @ R3 @ R4 @ R5

        alpha = np.arctan2(R_final[1, 0], R_final[1, 1])  # yaw
        beta = np.arcsin(R_final[1, 2])  # pitch
        gamma = -np.arctan(R_final[0, 2] / R_final[2, 2])  # roll

        return np.degrees(alpha), np.degrees(beta), np.degrees(gamma)

    def geographic_to_cartesian(self, BG, LG, HG):
        # Convert geographic coordinates to Cartesian coordinates
        """大地纬度：B-geo_lat, 大地经度：H-geo_lng"""
        BG, LG = np.radians(BG), np.radians(LG)
        # a = 6378137.0  # Earth's equatorial radius in meters
        # f = 1 / 298.257223563  # Flattening factor
        # e2 = f * (2 - f)  # Square of the first eccentricity
        N = self.EARTH_EQUATORIAL_RADIUS / np.sqrt(
            1 - self.FIRST_ECCENTRICITY_SQUARED * np.sin(BG) ** 2
        )

        XG = (N + HG) * np.cos(BG) * np.cos(LG)
        YG = (N + HG) * np.cos(BG) * np.sin(LG)
        ZG = (N * (1 - self.FIRST_ECCENTRICITY_SQUARED) + HG) * np.sin(BG)

        return np.array([XG, YG, ZG])

    def pixel_to_physics(self, pixel_coords):
        # Convert pixel coordinates to physical camera coordinates
        u0 = self.K[0, 2] * 2
        v0 = self.K[1, 2] * 2
        d_x = 1 / self.K[0, 0]
        d_y = 1 / self.K[1, 1]

        u, v = pixel_coords
        x = (u - u0 / 2) * d_x
        y = (v - v0 / 2) * d_y

        return np.array([x, y])

    def compute_world_coordinates(
        self,
        pixel_coords,
        platform_yaw,
        platform_pitch,
        platform_roll,
        camera_yaw,
        camera_pitch,
        geo_lng,
        geo_lat,
        geo_altitude,
        f=50,
    ):
        # Compute the world coordinates of the pixel
        alpha, beta, gamma = self.compute_attitude_angles(
            platform_yaw, platform_pitch, platform_roll, camera_yaw, camera_pitch
        )
        x, y = self.pixel_to_physics(pixel_coords)  # 得到相机坐标系下的x, y
        obj_pixel_camera = np.array([x, f, y])

        R_cam2enu = (
            self.rotation_matrix("Y", gamma)
            @ self.rotation_matrix("X", beta)
            @ self.rotation_matrix("Z", alpha)
        )
        R_geo2enu = self.rotation_matrix("X", 90 - geo_lat) @ self.rotation_matrix(
            "Z", 90 + geo_lng
        )
        platform_ecef = self.geographic_to_cartesian(geo_lat, geo_lng, geo_altitude)

        obj_pixel_ecef = R_geo2enu @ R_cam2enu @ obj_pixel_camera + platform_ecef
        return obj_pixel_ecef, platform_ecef

    def solve_intersection_ellipsoid(self, P, Q):
        # Solve for the intersection of a line (P, Q) with the Earth's ellipsoid
        XP, YP, ZP = P
        XQ, YQ, ZQ = Q

        A = (
            ((XP - XQ) ** 2 / self.EARTH_EQUATORIAL_RADIUS**2)
            + ((YP - YQ) ** 2 / self.EARTH_EQUATORIAL_RADIUS**2)
            + ((ZP - ZQ) ** 2 / self.EARTH_POLAR_RADIUS**2)
        )
        B = 2 * (
            (XQ * (XP - XQ)) / self.EARTH_EQUATORIAL_RADIUS**2
            + (YQ * (YP - YQ)) / self.EARTH_EQUATORIAL_RADIUS**2
            + (ZQ * (ZP - ZQ)) / self.EARTH_POLAR_RADIUS**2
        )
        C = (
            (XQ**2 / self.EARTH_EQUATORIAL_RADIUS**2)
            + (YQ**2 / self.EARTH_EQUATORIAL_RADIUS**2)
            + (ZQ**2 / self.EARTH_POLAR_RADIUS**2)
            - 1
        )

        discriminant = B**2 - 4 * A * C
        if discriminant < 0:
            raise ValueError("The line does not intersect the ellipsoid.")

        t1 = (-B + np.sqrt(discriminant)) / (2 * A)
        t2 = (-B - np.sqrt(discriminant)) / (2 * A)

        intersection1 = Q + t1 * (P - Q)
        intersection2 = Q + t2 * (P - Q)

        return (
            intersection1
            if np.linalg.norm(intersection1 - Q) < np.linalg.norm(intersection2 - Q)
            else intersection2
        )

    def ecef_to_geodetic(self, X, Y, Z, tol=1e-4, max_iter=1000):
        # Convert ECEF coordinates to geodetic coordinates
        longitude = np.arctan2(Y, X)  # Longitude

        p = np.sqrt(X**2 + Y**2)
        latitude = np.arctan2(
            Z, p * (1 - self.FIRST_ECCENTRICITY_SQUARED)
        )  # Initial latitude

        for _ in range(max_iter):
            sin_latitude = np.sin(latitude)
            N = self.EARTH_EQUATORIAL_RADIUS / np.sqrt(
                1 - self.FIRST_ECCENTRICITY_SQUARED * sin_latitude**2
            )
            h = p / np.cos(latitude) - N
            new_latitude = np.arctan2(
                Z + self.FIRST_ECCENTRICITY_SQUARED * N * sin_latitude, p
            )
            if np.abs(new_latitude - latitude) < tol:
                latitude = new_latitude
                break
            latitude = new_latitude
        else:
            raise RuntimeError("Iteration did not converge.")

        return np.degrees(latitude), np.degrees(longitude), h

    def locate_target(
        self,
        pixel_coords,
        platform_yaw,
        platform_pitch,
        platform_roll,
        camera_yaw,
        camera_pitch,
        geo_lng,
        geo_lat,
        geo_altitude,
        f=50,
    ):
        # Full process to locate the target
        obj_pixel_ecef, platform_ecef = self.compute_world_coordinates(
            pixel_coords,
            platform_yaw,
            platform_pitch,
            platform_roll,
            camera_yaw,
            camera_pitch,
            geo_lng,
            geo_lat,
            geo_altitude,
            f,
        )
        target_position = self.solve_intersection_ellipsoid(
            obj_pixel_ecef, platform_ecef
        )
        latitude, longitude, _ = self.ecef_to_geodetic(*target_position)
        return latitude, longitude


if __name__ == "__main__":
    # 0.015是像元尺寸, 640*512是图像分辨率
    K = np.array(
        [[1 / 0.015, 0, 640 / 2], [0, 1 / 0.015, 512 / 2], [0, 0, 1]]
    )  # Camera intrinsic matrix

    locator = TargetLocator(K)

    # Parameters for each calculation
    platform_yaw, platform_pitch, platform_roll = (
        -113.46,
        -0.22,
        2.09,
    )  # 航向角是246.54（0~360） 转换到了-180~+180范围
    camera_yaw, camera_pitch = 36.88, 1.82
    geo_lat, geo_lng, geo_altitude = 38.864295959, 121.640563965, 86.9  # 高度单位是m
    f = 50  # Focal length in mm

    # Pixel coordinates
    pixel_coords = (240, 336)

    # Locate target for the given pixel
    latitude, longitude = locator.locate_target(
        pixel_coords,
        platform_yaw,
        platform_pitch,
        platform_roll,
        camera_yaw,
        camera_pitch,
        geo_lng,
        geo_lat,
        geo_altitude,
        f,
    )

    print(f"Target latitude: {latitude}°, longitude: {longitude}°")
