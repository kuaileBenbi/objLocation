import numpy as np

"""
step1 : 求光轴姿态角
step2 : 利用相机内参矩阵将目标像素坐标转换为相机坐标
step3 : 利用定位平台经纬高，计算定位平台的地心坐标
step4 : 利用光轴姿态角求出光轴姿态变换矩阵、定位平台经纬度、定位平台的地心坐标，求出目标像素的地心坐标
step5 : 利用定位平台地心坐标、目标像素的地心坐标，两者连线与地球的椭球模型交点为目标的地心坐标
step6 ：将目标地心坐标转换为经纬度
"""


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


platform_yaw, platform_pitch, platform_roll = -113.46, -0.22, 2.09
camera_yaw, camera_pitch = 36.88, 1.82

R1 = rotation_matrix("X", camera_pitch)  # Step 1: Pitch around XC axis
R2 = rotation_matrix("Z", camera_yaw)  # Step 2: Yaw around ZC axis
R3 = rotation_matrix("Y", platform_roll)  # Step 3: Roll around YC axis
R4 = rotation_matrix("X", platform_pitch)  # Step 4: Pitch around XC axis
R5 = rotation_matrix("Z", platform_yaw)  # Step 5: Yaw around ZC axis
R_final = R1 @ R2 @ R3 @ R4 @ R5

alpha = np.arctan2(R_final[1, 0], R_final[1, 1])  # yaw -180~+180
beta = np.arcsin(R_final[1, 2])  # pitch -90~+90
gamma = -np.arctan(R_final[0, 2] / R_final[2, 2])  # roll -90~+90

print(f"yaw:{np.degrees(alpha)}, pitch:{np.degrees(beta)}, roll:{np.degrees(gamma)}")


def geographic_to_cartesian(BG, LG, HG):
    # https://blog.csdn.net/why1472587/article/details/128190538
    # BG, LG, HG -> 纬度、经度、高度
    BG, LG = np.radians(BG), np.radians(LG)
    a = 6378137.0  # Earth's equatorial radius in meters
    f = 1 / 298.257223563  # 扁率
    e2 = f * (2 - f)  # Square of the Earth's first eccentricity
    # Calculate the prime vertical radius of curvature
    N = a / np.sqrt(1 - e2 * np.sin(BG) ** 2)

    # Compute Cartesian coordinates
    XG = (N + HG) * np.cos(BG) * np.cos(LG)
    YG = (N + HG) * np.cos(BG) * np.sin(LG)
    ZG = (N * (1 - e2) + HG) * np.sin(BG)

    return np.array([XG, YG, ZG])


geo_lng = 38.864295959
geo_lat = 121.640563965
geo_altitude = 86.9

XG, YG, ZG = geographic_to_cartesian(geo_lng, geo_lat, geo_altitude)
PG = np.array([XG, YG, ZG])

print(f"camera Pw was {XG, YG, ZG}")


def pixel_to_physics(pixel_coords, K):
    u0 = K[0, 2] * 2
    v0 = K[1, 2] * 2
    d_x = 1 / K[0, 0]
    d_y = 1 / K[1, 1]

    u, v = pixel_coords

    x = (u - u0 / 2) * d_x
    y = (v - v0 / 2) * d_y

    return np.array([x, y])


K = np.array(
    [[1 / 0.015, 0, 640 / 2], [0, 1 / 0.015, 512 / 2], [0, 0, 1]]  # fx, u0  # fy, v0
)  # 缩放因子

x, y = pixel_to_physics((240, 336), K)
print(f"physics coordinates: {x,y}")
# P(Xc, Yc, Zc) -> P(x, f, y), f=50mm
f = 50
Pc = np.array([x, f, y])
print(f"camera coordinates: {Pc}")


R_cam2enu = (
    rotation_matrix("Y", gamma)
    @ rotation_matrix("X", beta)
    @ rotation_matrix("Z", alpha)
)
R_B = rotation_matrix("X", 90 - geo_lat) @ rotation_matrix("Z", 90 + geo_lng)

XW, YW, ZW = R_B @ R_cam2enu @ Pc + PG

PW = np.array([XW, YW, ZW])

print(f"pixel PW was: {PW}")


def solve_intersection_ellipsoid(P, Q, a, b):
    # P 和 Q 为直线上两点的坐标, a 为赤道半径, b 为极半径
    XP, YP, ZP = P
    XG, YG, ZG = Q

    # 计算 A, B, C 系数
    A = ((XP - XG) ** 2 / a**2) + ((YP - YG) ** 2 / a**2) + ((ZP - ZG) ** 2 / b**2)
    B = 2 * (
        (XG * (XP - XG)) / a**2 + (YG * (YP - YG)) / a**2 + (ZG * (ZP - ZG)) / b**2
    )
    C = (XG**2 / a**2) + (YG**2 / a**2) + (ZG**2 / b**2) - 1

    # 求解二次方程 At^2 + Bt + C = 0
    discriminant = B**2 - 4 * A * C
    if discriminant < 0:
        raise ValueError("直线与椭球体没有交点")

    # 计算 t 的两个解
    t1 = (-B + np.sqrt(discriminant)) / (2 * A)
    t2 = (-B - np.sqrt(discriminant)) / (2 * A)

    # 计算两个交点
    intersection1 = Q + t1 * (P - Q)
    intersection2 = Q + t2 * (P - Q)

    # 返回较靠近 Q 的交点
    if np.linalg.norm(intersection1 - Q) < np.linalg.norm(intersection2 - Q):
        return intersection1
    else:
        return intersection2


P = PW  # 点 P 的坐标
Q = PG  # 平台 Q 的坐标

a = 6378137.0  # 地球赤道半径（米）
b = 6356752.3  # 地球极半径（米）

target_position = solve_intersection_ellipsoid(P, Q, a, b)
print("目标位置:", target_position)

# WGS84 椭球参数
a = 6378137.0  # 赤道半径，单位：米
b = 6356752.3  # 极半径，单位：米
e2 = (a**2 - b**2) / a**2  # 第一偏心率的平方


def ecef_to_geodetic(X, Y, Z, tol=1e-12, max_iter=1000):
    """
    将地心直角坐标 (X, Y, Z) 转换为大地经纬度 (纬度, 经度, 高度)。
    :param X: 地心坐标系中的 X 坐标，单位：米
    :param Y: 地心坐标系中的 Y 坐标，单位：米
    :param Z: 地心坐标系中的 Z 坐标，单位：米
    :param tol: 收敛容差
    :param max_iter: 最大迭代次数
    :return: (纬度 B， 经度 N， 高度 h)
    """
    # 计算经度 N（longitude）
    N = np.arctan2(Y, X)  # 经度 = arctan(Y / X)

    # 初始纬度估计值 B（latitude）
    p = np.sqrt(X**2 + Y**2)  # 横截面上的投影距离
    B = np.arctan2(Z, p * (1 - e2))  # 初始纬度估计

    # 牛顿迭代计算纬度 B
    for _ in range(max_iter):
        sinB = np.sin(B)
        N_radius = a / np.sqrt(1 - e2 * sinB**2)  # 卯酉圈半径
        h = p / np.cos(B) - N_radius  # 高度
        new_B = np.arctan2(Z + N_radius * e2 * sinB, p)  # 更新纬度

        # 检查收敛
        if np.abs(new_B - B) < tol:
            B = new_B
            break
        B = new_B

    else:
        raise RuntimeError("迭代未收敛")

    # 返回的纬度 B 以弧度表示，可以转换为度数
    return np.degrees(B), np.degrees(N), h


X, Y, Z = target_position
latitude, longitude, _ = ecef_to_geodetic(X, Y, Z)
print(f"纬度: {latitude}°, 经度: {longitude}°")
