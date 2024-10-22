# **TargetLocator 类**

## **概述**

`TargetLocator` 类用于根据图像中目标物体的像素坐标计算其地理位置（纬度和经度），考虑了光电吊舱和平台的姿态、相机参数以及平台的地理位置。

## **计算的关键步骤**

1. **相机光轴的姿态角：** 计算相机光轴相对于东北天坐标系的偏航角、俯仰角、横滚角。由光电吊舱的方位角、高低角和定位平台的航向角、俯仰角、横滚角共同决定姿态矩阵。

   - 定位平台的姿态角由惯导获取，符合东北天（ENU）坐标系，吊舱姿态角由编码器、陀螺仪等获取。吊舱姿态相对于载体。

   - 载体坐标系（即动态坐标系，b系）采用右前上。机体前进方向与Y轴正半轴重合，X轴正半轴指向机体右边。假设系统在初始时b系与ENU重合，吊舱的航向、俯仰为0°时与Y轴正半轴重合。

   - 旋转顺序遵循偏航（Z）-俯仰（X）-横滚（Y）。
  
2. **像素到世界坐标的转换：** 利用相机的内参和姿态角，将像素坐标投影到3D空间中，并将其转换为地心地固坐标系（ECEF）中的世界坐标。

3. **与地球椭球的交点：** 世界坐标形成一条线，连接平台位置，找到该线与地球椭球模型的交点，即目标的地理坐标。（若有多个交点，选择距离定位平台最近的交点作为结果）

4. **大地坐标转换：** 最后，将交点的ECEF坐标转换为大地纬度和经度。

---

## **类和方法描述**

### **类: TargetLocator**

此类执行从像素坐标到目标定位所需的所有步骤。

#### **初始化**

```python
locator = TargetLocator(K)
```

`k` (3x3 numpy 矩阵): 相机内参矩阵，定义相机如何将世界3D点映射到2D像素坐标。包括像元尺寸`dx, dy`和主点`cx, cy`（光心，近似为图像宽/2，图像高/2）。

#### **方法 `compute_attitude_angles`**

```python
alpha, beta, gamma = compute_attitude_angles(platform_yaw, platform_pitch, platform_roll, camera_yaw, camera_pitch)
```

- **用途**: 计算相机相对于世界坐标系的组合姿态角（偏航角、俯仰角、横滚角）。这些角度表示在全球框架中相机的朝向，考虑平台和相机的姿态。

- **参数**:
   - `platform_yaw`, `platform_pitch`, `platform_roll` (float): 定位平台的偏航角、俯仰角和横滚角，单位为度。
   - `camera_yaw`, `camera_pitch` (float): 吊舱的偏航角和俯仰角，单位为度。

- **返回**: 

   - `alpha` (float): 偏航角，单位为度。
   - `beta` (float): 俯仰角，单位为度。
   - `gamma` (float): 横滚角，单位为度。

#### **方法 `geographic_to_cartesian`**

```python
geographic_to_cartesian(geo_lat, geo_lng, geo_altitude)
```

- **用途**: 将平台的地理坐标（纬度、经度、高度）转换为地心地固坐标系（ECEF）中的笛卡尔坐标。

- **参数**:
   - `geo_lat`(float): 平台的纬度，单位为度。
   - `geo_lng`(float): 平台的经度，单位为度。
   - `geo_altitude`(float): 平台的高度，单位为度。

- **返回**: 

   - 一个 numpy 数组 `[XG, YG, ZG]`，表示平台在ECEF系统中的笛卡尔坐标。

#### **方法 `pixel_to_physics`**

```python
pixel_to_physics(pixel_coords)
```

- **用途**: 利用相机内参矩阵K，将图像中的像素坐标转换为相机3D空间中的归一化物理坐标（单位为毫米）。

- **参数**:
   - `pixel_coords` (元组): 图像中的像素坐标 `(u, v)`。

- **返回**: 

   - 一个 numpy 数组 `[x, y]`，表示相机3D空间中的物理坐标。

#### **方法 `compute_world_coordinates`**

```python
compute_world_coordinates(pixel_coords, platform_yaw, platform_pitch, platform_roll, camera_yaw, camera_pitch, geo_lng, geo_lat, geo_altitude, f=50)
```

- **用途**: 根据像素坐标和平台/相机参数计算目标点的世界坐标（ECEF）。

- **参数**:
   - `pixel_coords` (元组): 图像中的像素坐标 `(u, v)`。
   - `platform_yaw, platform_pitch, platform_roll (float)`: 平台的姿态角，单位为度。
   - `camera_yaw, camera_pitch (float)`: 相机的姿态角，单位为度。
   - `geo_lng, geo_lat, geo_altitude (float)`: 平台的地理坐标。
   - `f` (float, 可选): 相机焦距，单位为毫米（默认: 50mm）。

- **返回**: 

   - `obj_pixel_ecef` (numpy 数组): 目标像素的世界坐标（ECEF）。
   - `platform_ecef` (numpy 数组): 平台的世界坐标（ECEF）。


#### **方法 `solve_intersection_ellipsoid`**

```python
solve_intersection_ellipsoid(P, Q)
```

- **用途**: 解决点 `P` 和 `Q` 所在线与地球椭球（WGS84模型）的交点。

- **参数**:
   - `P` (numpy 数组): 目标像素的世界坐标。
   - `Q`(numpy 数组): 平台的世界坐标。
   - `a` (float, 可选): 地球赤道半径，单位为米（默认: 6378137.0米）。
   - `b` (float, 可选): 地球极半径，单位为米（默认: 6356752.3米）。

- **返回**: 

   - 离Q最近的交点坐标（ECEF）。


#### **方法 `ecef_to_geodetic`**

```python
ecef_to_geodetic(X, Y, Z, tol=1e-4, max_iter=1000)
```

- **用途**: 将ECEF笛卡尔坐标转换为大地坐标（纬度、经度、高度）。

- **参数**:
   - `X, Y, Z`(float): ECEF 坐标。
   - `tol`(float, 可选): 纬度计算的收敛容差（默认: 1e-4）。
   - `max_iter` (int, 可选): 最大迭代次数（默认: 1000）。

- **返回**: 

   - `latitude`(float): 纬度，单位为度。
   - `longitude`(float): 经度，单位为度。

#### **方法 `locate_target`**

```python
locate_target(pixel_coords, platform_yaw, platform_pitch, platform_roll, camera_yaw, camera_pitch, geo_lng, geo_lat, geo_altitude, f=50)
```

- **用途**: 主方法，根据图像中的像素坐标定位目标。该方法通过以下步骤计算目标的纬度和经度：
   1. 计算目标像素的世界坐标。
   2. 解决目标像素-相机光心的世界坐标的连线与地球椭球的交点。
   3. 将交点坐标转换为地理坐标。

- **参数**:
   - `pixel_coords` (元组): 图像中的像素坐标 `(u, v)`。
   - `platform_yaw, platform_pitch, platform_roll`(float): 平台的姿态角，单位为度。
   - `camera_yaw, camera_pitch` (float): 相机的姿态角，单位为度。
   - `geo_lng, geo_lat, geo_altitude`(float): 平台的地理坐标。
   - `f`(float, 可选): 相机焦距，单位为毫米（默认: 50mm）。


- **返回**: 

   - `latitude`(float): 纬度，单位为度。
   - `longitude`(float): 经度，单位为度。

---

# **使用方法**

```python
K = np.array([
    [1 / 0.015, 0, 640 / 2],
    [0, 1 / 0.015, 512 / 2],
    [0, 0, 1]
])  # 相机内参矩阵

locator = TargetLocator(K)

# 每次计算所需的参数
platform_yaw, platform_pitch, platform_roll = -113.46, -0.22, 2.09
camera_yaw, camera_pitch = 36.88, 1.82
geo_lng, geo_lat, geo_altitude = 38.864295959, 121.640563965, 86.9
f = 50  # 焦距，单位为mm

# 像素坐标
pixel_coords = (240, 336)

# 定位目标
latitude, longitude = locator.locate_target(
    pixel_coords,
    platform_yaw, platform_pitch, platform_roll,
    camera_yaw, camera_pitch,
    geo_lng, geo_lat, geo_altitude,
    f
)

print(f"目标纬度: {latitude}°，目标经度: {longitude}°")
```

# **工作流程解释**

1. 平台和相机姿态：

   - 使用平台的偏航角、俯仰角和横滚角以及相机的偏航角和俯仰角，计算相机的整体朝向。

2. 像素坐标：

   - 将图像中的像素坐标转换为相机3D空间中的物理坐标，再通过一系列旋转和转换矩阵（相机朝向矩阵、经纬度旋转矩阵、平台经纬度平移坐标）将其转换为地心地固坐标系（ECEF）中的世界坐标。

3. 目标定位：

   - 根据目标像素与光心的世界坐标两点确定的直线，找到其与地球椭球的交点，并将交点转换为地理坐标，即目标的纬度和经度。