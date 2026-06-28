# Cartographer 2D 长直隧道退化场景下的人工锚点约束方案

## 摘要

长直隧道、长走廊、管廊等场景在 2D 激光 SLAM 中容易出现几何退化。机器人沿隧道轴向运动时，激光雷达观测到的左右墙面几何形态高度重复，scan matching 对轴向位移的约束较弱。即使系统配备 IMU 和轮式里程计，Cartographer 2D 建图结果仍可能出现轴向尺度压缩，例如实际长度 100 m 的隧道最终建成 40 m 左右。

该问题可以通过引入外部绝对位置约束缓解。本文针对以下工程条件，给出两种可实施方案：

- 隧道地面每 10 m 使用精密仪器打点，锚点位置可视为绝对准确。
- 建图起点为 global/map 原点。
- 机器人启动前与隧道轴线严格对齐，初始 `x = 0`、`y = 0`、`yaw = 0`。
- 所有锚点在 global 坐标系下满足 `y = 0`、`yaw = 0`，仅 `x` 不同。
- 锚点约束可作为强约束。
- 使用 Cartographer ROS，可发布 ROS 话题。

本文重点分析两种方案：

1. **虚拟 Landmark 方案**：将每个 10 m 点建模为一个固定 landmark，当机器人到达该点时发布 landmark 观测。
2. **人工 fixed-frame/GPS 方案**：将每个 10 m 点建模为机器人在 fixed frame 下的绝对位置观测，通过 Cartographer ROS 已有 `NavSatFix` 入口注入后端。

结论如下：

- 在不修改 Cartographer 代码的前提下，推荐使用 **人工 fixed-frame/GPS 方案**。
- 纯发布 `/landmark` 话题只能添加 landmark 观测，不能设置 landmark 的 global pose 且 frozen，因此不能完整表达“固定绝对锚点”。
- 若允许修改代码，虚拟 landmark 方案可以实现得更符合“地面锚点”语义：启动时预设所有 landmark 的 global pose，并将其固定为常量，运行时到点发布单位相对观测。

---

## 1. 场景与问题定义

### 1.1 传感器配置

目标系统使用：

- 2D 激光雷达
- IMU
- 轮式里程计
- Cartographer ROS
- Cartographer 2D SLAM

### 1.2 场景特点

目标环境为一条长约 100 m 的笔直隧道。隧道轴线定义为 global/map 坐标系的 `+x` 方向，起点定义为：

```text
x: 隧道前进方向
y: 隧道横向方向
z: 垂直地面方向

起点:
  x = 0
  y = 0
  yaw = 0
```

地面每 10 m 存在一个人工精密打点锚点：

```text
anchor_0    = (0,   0, 0)
anchor_10   = (10,  0, 0)
anchor_20   = (20,  0, 0)
...
anchor_100  = (100, 0, 0)
```

其中第三维 `0` 表示 yaw。

### 1.3 退化表现

长直隧道中，2D 激光匹配对横向位置和 yaw 通常仍有较强约束，但对轴向平移的可观测性较弱。典型现象如下：

```text
真实环境长度:
  100 m

建图结果长度:
  可能只有 40 m、50 m 或其他明显压缩尺度
```

产生该现象的核心原因是：

- 左右墙面在长距离上几何相似。
- 轴向移动一定距离后，激光观测仍可能与之前的局部地图匹配良好。
- scan matching 可以通过错误的轴向位移解释当前扫描。
- 里程计若权重不足或存在误差，无法完全抵消 scan matching 退化。
- pose graph 优化在全局上缺少绝对尺度约束。

因此，需要引入额外观测来表达：

```text
机器人在某些时刻确实位于 global x = 10, 20, ..., 100 m 的位置。
```

---

## 2. Cartographer 后端约束相关背景

### 2.1 子图-节点约束

Cartographer 2D pose graph 中，常规 scan matching 约束主要连接：

```text
Submap i
Trajectory Node j
```

残差形式可概括为：

```text
观测到的节点相对子图位姿
-
由当前 global 子图位姿和 global 节点位姿推算出的相对位姿
```

公式表示：

```text
e_ij = zbar_ij - inverse(T_G_Si) * T_G_Nj
```

其中：

- `T_G_Si`：子图 `i` 在 global 坐标系下的位姿。
- `T_G_Nj`：节点 `j` 在 global 坐标系下的位姿。
- `zbar_ij`：scan matching 或节点插入关系得到的节点相对子图观测。

在长直隧道中，`zbar_ij` 的轴向信息可能不可靠，最终导致轨迹和地图被压缩。

### 2.2 Landmark 约束

Cartographer 的 landmark 约束连接：

```text
Trajectory Node / tracking frame
Landmark global pose
```

一个 landmark observation 表示：

```text
某个时间 t，tracking_frame 观测到某个 landmark，
观测值为 tracking_frame 与 landmark 之间的相对位姿。
```

Cartographer ROS 侧消息为：

```text
cartographer_ros_msgs/LandmarkList
```

其条目 `LandmarkEntry` 为：

```text
string id
geometry_msgs/Pose tracking_from_landmark_transform
float64 translation_weight
float64 rotation_weight
```

当机器人与锚点完全重合时，应发布单位位姿：

```text
translation = (0, 0, 0)
rotation = (x=0, y=0, z=0, w=1)
```

### 2.3 Fixed-frame/GPS 约束

Cartographer 2D 后端还支持 fixed-frame pose 约束。ROS 标准入口通常是：

```text
/fix
sensor_msgs/NavSatFix
```

`NavSatFix` 数据会被转换为 fixed-frame pose 数据，并加入后端优化。该约束的作用是：

```text
在某个时间 t，机器人 tracking_frame 在某个固定坐标系下的位置应该接近外部观测值。
```

这与人工地面锚点的语义高度一致：

```text
机器人到达 anchor_50 时，当前 tracking_frame 的 global x 应该为 50 m。
```

---

## 3. 方案一：虚拟 Landmark 方案

### 3.1 基本思想

将地面每个 10 m 锚点建模为一个 landmark：

```text
anchor_0
anchor_10
anchor_20
...
anchor_100
```

每个 landmark 的 global 位姿已知：

```text
anchor_0:
  global pose = x: 0, y: 0, yaw: 0

anchor_10:
  global pose = x: 10, y: 0, yaw: 0

...

anchor_100:
  global pose = x: 100, y: 0, yaw: 0
```

运行过程中，当机器人到达某个锚点并与其重合时，发布：

```text
id = anchor_xx
tracking_from_landmark_transform = Identity
```

如果 landmark 的 global pose 被固定为常量，那么该观测等价于：

```text
当前 tracking_frame 的 global pose 应该与 anchor_xx 的 global pose 重合。
```

### 3.2 数学原理

设：

```text
T_G_T(t)  = 观测时刻 tracking_frame 在 global 下的位姿
T_G_L     = landmark 在 global 下的固定已知位姿
T_T_L^obs = tracking_frame 到 landmark 的观测相对位姿
```

后端预测值为：

```text
T_T_L^pred = inverse(T_G_T(t)) * T_G_L
```

landmark 残差为：

```text
e = T_T_L^obs - T_T_L^pred
```

当机器人与锚点重合时：

```text
T_T_L^obs = Identity
```

如果 `T_G_L = (50, 0, 0)` 且 `T_T_L^obs = Identity`，优化器会推动：

```text
T_G_T(t) ≈ (50, 0, 0)
```

### 3.3 关键前提

虚拟 landmark 方案必须满足：

```text
每个 landmark 的 global pose 已知。
每个 landmark 的 global pose 在优化中固定，不允许被优化器移动。
```

如果只发布 `/landmark` 观测，但不固定 landmark global pose，则后端会同时优化：

```text
机器人轨迹
landmark 位置
```

此时 landmark 会跟随轨迹一起移动，不能提供绝对尺度约束。

### 3.4 不修改代码时的可行性

Cartographer ROS 已有 `/landmark` 话题入口：

```text
topic:
  /landmark

type:
  cartographer_ros_msgs/LandmarkList
```

需要在 lua 中开启：

```lua
use_landmarks = true
landmarks_sampling_ratio = 1.
```

但是，普通 Cartographer ROS node 没有提供一个标准 ROS 话题或服务用于设置：

```text
SetLandmarkPose(id, global_pose, frozen=true)
```

因此，在不修改代码的前提下，纯 `/landmark` 发布只能完成“观测输入”，不能完成“固定绝对锚点”的设置。

结论：

```text
不修改代码时，虚拟 landmark 方案不推荐作为主方案。
```

### 3.5 允许修改代码时的实现方式

如果允许修改 Cartographer ROS 代码，虚拟 landmark 方案是可完整实现的。

#### 3.5.1 修改目标

实现目标：

```text
启动 Cartographer 时预先注册 anchor_0...anchor_100 的 global pose。
将这些 landmark 设置为 frozen。
运行时通过 /landmark 发布观测。
```

#### 3.5.2 相关核心接口

Cartographer 内部已有接口：

```cpp
PoseGraphInterface::SetLandmarkPose(
    const std::string& landmark_id,
    const transform::Rigid3d& global_pose,
    const bool frozen = false)
```

2D 实现位置：

```text
cartographer/cartographer/mapping/internal/2d/pose_graph_2d.cc
```

其作用是：

```text
设置某个 landmark 的 global pose。
当 frozen = true 时，该 landmark 在优化中作为常量。
```

因此，不需要修改 Cartographer 后端优化数学模型，只需要在 ROS 层提供配置或服务调用该接口。

#### 3.5.3 修改方案 A：在 ROS node 启动时从配置加载固定锚点

适用场景：

```text
锚点位置固定，数量固定，例如 0,10,...,100 m。
每次建图都使用同一组锚点。
```

建议新增配置项，例如在 lua 中增加：

```lua
fixed_landmarks = {
  { id = "anchor_0",   x = 0.,   y = 0., yaw = 0. },
  { id = "anchor_10",  x = 10.,  y = 0., yaw = 0. },
  { id = "anchor_20",  x = 20.,  y = 0., yaw = 0. },
  { id = "anchor_30",  x = 30.,  y = 0., yaw = 0. },
  { id = "anchor_40",  x = 40.,  y = 0., yaw = 0. },
  { id = "anchor_50",  x = 50.,  y = 0., yaw = 0. },
  { id = "anchor_60",  x = 60.,  y = 0., yaw = 0. },
  { id = "anchor_70",  x = 70.,  y = 0., yaw = 0. },
  { id = "anchor_80",  x = 80.,  y = 0., yaw = 0. },
  { id = "anchor_90",  x = 90.,  y = 0., yaw = 0. },
  { id = "anchor_100", x = 100., y = 0., yaw = 0. },
}
```

需要修改的代码位置：

```text
cartographer_ros/cartographer_ros/cartographer_ros/trajectory_options.h
cartographer_ros/cartographer_ros/cartographer_ros/trajectory_options.cc
cartographer_ros/cartographer_ros/cartographer_ros/map_builder_bridge.cc
cartographer_ros/cartographer_ros/cartographer_ros/map_builder_bridge.h
cartographer_ros/cartographer_ros/cartographer_ros/node.cc
```

推荐实现步骤：

1. 在 `trajectory_options.h` 中新增一个结构体保存 fixed landmark 配置：

   ```cpp
   struct FixedLandmarkPose {
     std::string id;
     double x;
     double y;
     double yaw;
   };
   ```

   并在 `TrajectoryOptions` 中增加：

   ```cpp
   std::vector<FixedLandmarkPose> fixed_landmarks;
   ```

2. 在 `trajectory_options.cc` 的 `CreateTrajectoryOptions()` 中解析 lua 的 `fixed_landmarks` 表。

3. 在轨迹启动后，调用 pose graph 的：

   ```cpp
   map_builder_->pose_graph()->SetLandmarkPose(
       landmark_id,
       transform::Rigid3d(
           Eigen::Vector3d(x, y, 0.),
           transform::RollPitchYaw(0., 0., yaw)),
       true /* frozen */);
   ```

4. 运行时继续通过 `/landmark` 发布观测。

注意事项：

- 需要保证设置 fixed landmark 的时机早于这些 landmark 的观测进入优化。
- `frozen = true` 是关键，否则 landmark 仍可能被优化器移动。
- 此方案需要修改并重新编译 `cartographer_ros`。

#### 3.5.4 修改方案 B：新增 ROS service 设置 fixed landmark

适用场景：

```text
锚点集合可能变化。
希望外部程序在建图启动后动态设置锚点。
```

建议新增服务，例如：

```text
SetLandmarkPose.srv
```

请求：

```text
string id
geometry_msgs/Pose global_pose
bool frozen
---
bool success
string message
```

需要修改的代码位置：

```text
cartographer_ros/cartographer_ros_msgs/srv/SetLandmarkPose.srv
cartographer_ros/cartographer_ros_msgs/CMakeLists.txt
cartographer_ros/cartographer_ros/cartographer_ros/node.h
cartographer_ros/cartographer_ros/cartographer_ros/node.cc
cartographer_ros/cartographer_ros/cartographer_ros/map_builder_bridge.h
cartographer_ros/cartographer_ros/cartographer_ros/map_builder_bridge.cc
```

推荐实现步骤：

1. 在 `cartographer_ros_msgs/srv/` 下新增 `SetLandmarkPose.srv`。

2. 修改 `cartographer_ros_msgs/CMakeLists.txt`，将服务加入 `add_service_files()`。

3. 在 `Node` 中注册服务，例如：

   ```cpp
   ros::ServiceServer set_landmark_pose_service_;
   ```

4. 在服务回调中调用：

   ```cpp
   map_builder_bridge_.SetLandmarkPose(id, global_pose, frozen);
   ```

5. 在 `MapBuilderBridge` 中封装：

   ```cpp
   map_builder_->pose_graph()->SetLandmarkPose(id, global_pose, frozen);
   ```

6. 启动后先调用服务设置所有锚点：

   ```text
   anchor_0    -> (0,   0, 0), frozen = true
   anchor_10   -> (10,  0, 0), frozen = true
   ...
   anchor_100  -> (100, 0, 0), frozen = true
   ```

7. 建图过程中到点发布 `/landmark`。

该方案扩展性更好，但改动比配置加载方式更多。

### 3.6 Landmark 观测发布步骤

无论固定 landmark 通过哪种方式设置，运行时观测发布逻辑一致。

#### 3.6.1 Cartographer 配置

lua 中设置：

```lua
use_landmarks = true
landmarks_sampling_ratio = 1.
```

#### 3.6.2 发布话题

话题：

```text
/landmark
```

类型：

```text
cartographer_ros_msgs/LandmarkList
```

到达 `x = 50 m` 锚点时发布：

```text
header.stamp = 当前时间
header.frame_id = tracking_frame 或可 TF 到 tracking_frame 的传感器 frame

landmarks[0].id = "anchor_50"
landmarks[0].tracking_from_landmark_transform.position.x = 0
landmarks[0].tracking_from_landmark_transform.position.y = 0
landmarks[0].tracking_from_landmark_transform.position.z = 0
landmarks[0].tracking_from_landmark_transform.orientation.x = 0
landmarks[0].tracking_from_landmark_transform.orientation.y = 0
landmarks[0].tracking_from_landmark_transform.orientation.z = 0
landmarks[0].tracking_from_landmark_transform.orientation.w = 1
landmarks[0].translation_weight = 1e3 ~ 1e5
landmarks[0].rotation_weight = 1e3 ~ 1e5
```

如果只希望强约束位置，不希望强约束 yaw，可将：

```text
rotation_weight
```

设置得较低。但在当前场景中，机器人 yaw 与隧道轴线严格对齐，锚点 yaw 也为 0，因此 rotation 约束可以保留。

#### 3.6.3 发布频率

到达每个锚点后，不建议只发布一帧。建议静止或低速通过时连续发布：

```text
频率: 5 Hz ~ 20 Hz
持续: 0.5 s ~ 2 s
```

目的：

- 避免采样器、时间同步或回调延迟导致观测未进入后端。
- 让后端能在相邻 trajectory node 间插值出更准确的观测时刻。

### 3.7 Landmark 方案优缺点

优点：

- 语义上与“地面固定锚点”一致。
- 每个锚点有独立 ID，便于可视化和调试。
- 可以约束平移和旋转。
- 若 landmark frozen，实现后可以形成非常强的绝对锚点约束。

缺点：

- 普通 Cartographer ROS 话题只能发布观测，不能设置 fixed/frozen landmark global pose。
- 不修改代码时不能完整表达“锚点 global 坐标已知且固定”。
- 若未 frozen，landmark 会被优化器一起移动，不能可靠修正地图压缩。
- 修改 ROS 接口后需要重新编译并维护自定义分支。

---

## 4. 方案二：人工 fixed-frame/GPS 方案

### 4.1 基本思想

将地面锚点观测视为一种外部绝对位置观测：

```text
机器人到达 anchor_10 时，tracking_frame 的 fixed/global 坐标为 x = 10。
机器人到达 anchor_20 时，tracking_frame 的 fixed/global 坐标为 x = 20。
...
机器人到达 anchor_100 时，tracking_frame 的 fixed/global 坐标为 x = 100。
```

Cartographer ROS 已支持 `NavSatFix` 输入，因此可将人工锚点转换成伪 GPS 数据发布到：

```text
/fix
sensor_msgs/NavSatFix
```

该方案无需修改 Cartographer 代码。

### 4.2 数学原理

fixed-frame/GPS 约束本质上表达：

```text
某个时间 t，机器人在固定坐标系下的位置应该接近外部观测值。
```

在本场景中，外部观测值由人工锚点给出：

```text
z_t = (x_anchor, 0)
```

后端优化时，轨迹节点插值得到观测时刻的 tracking frame global pose：

```text
T_G_T(t)
```

固定坐标观测约束会推动：

```text
translation(T_G_T(t)) ≈ (x_anchor, 0)
```

因此，当 `x = 0, 10, ..., 100` 的观测陆续加入后，pose graph 在轴向尺度上获得强约束，地图压缩会被显著抑制。

### 4.3 为什么该方案更适合不改代码场景

人工锚点的真实语义是：

```text
当前机器人在全局坐标系中的位置已知。
```

这与 fixed-frame/GPS 约束完全一致。

相比之下，landmark 方案需要两步：

```text
1. 设置固定 landmark 的 global pose。
2. 发布 tracking 到 landmark 的相对观测。
```

在普通 Cartographer ROS 中，第 1 步没有直接话题接口；而 fixed-frame/GPS 方案只需要发布 `/fix`。

### 4.4 Cartographer 配置

在 lua 中启用 nav sat：

```lua
use_nav_sat = true
fixed_frame_pose_sampling_ratio = 1.
```

不需要启用 landmark：

```lua
use_landmarks = false
```

设置 fixed-frame 观测权重：

```lua
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e3
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 0.
```

建议从 `1e3` 开始。如果地图仍明显压缩，可逐步提高：

```lua
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e4
```

或：

```lua
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e5
```

不建议一开始设置极端大值，以避免优化数值条件变差。

### 4.5 人工 x 坐标到 NavSatFix 的转换

`NavSatFix` 使用经纬度，而地面锚点为局部米制坐标。因此需要定义一个虚拟经纬度原点：

```text
lat0 = 30.000000000 deg
lon0 = 120.000000000 deg
alt0 = 0 m
```

将隧道 `+x` 方向映射到局部东向：

```text
east = x
north = 0
```

小范围近似转换：

```text
delta_lat = north / 111320.0
delta_lon = east / (111320.0 * cos(lat0))

lat = lat0 + delta_lat
lon = lon0 + delta_lon
```

由于隧道长度仅 100 m，该近似精度足够。

示例：`lat0 = 30 deg` 时：

```text
cos(30 deg) ≈ 0.8660254
1 deg longitude ≈ 111320 * 0.8660254 ≈ 96405 m
```

因此：

```text
x = 0 m:
  lat = 30.000000000
  lon = 120.000000000

x = 10 m:
  lat = 30.000000000
  lon ≈ 120.00010373

x = 100 m:
  lat = 30.000000000
  lon ≈ 120.00103730
```

### 4.6 NavSatFix 消息字段建议

话题：

```text
/fix
```

类型：

```text
sensor_msgs/NavSatFix
```

消息建议：

```text
header.stamp = 当前 ROS 时间
header.frame_id = "gps"

status.status = sensor_msgs/NavSatStatus.STATUS_FIX
status.service = sensor_msgs.NavSatStatus.SERVICE_GPS

latitude = lat(x)
longitude = lon(x)
altitude = 0

position_covariance[0] = 1e-4
position_covariance[4] = 1e-4
position_covariance[8] = 1e-4
position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN
```

协方差字段主要用于表达数据可信度。Cartographer 中该约束的实际强度主要由 lua 的 fixed-frame 权重控制。

### 4.7 发布流程

推荐启动流程：

```text
1. 将机器人放置在 anchor_0。
2. 机器人 heading 与隧道 +x 方向完全一致。
3. 启动 TF、robot_state_publisher、雷达、IMU、里程计等基础节点。
4. 启动 Cartographer。
5. 在起点连续发布若干帧 x = 0 的 fake NavSatFix。
6. 开始移动建图。
7. 到达 x = 10 m 锚点时，人工触发发布 x = 10 的 fake NavSatFix。
8. 到达 x = 20 m 锚点时，人工触发发布 x = 20 的 fake NavSatFix。
9. 依次发布至 x = 100 m。
10. 等待 pose graph 优化完成，检查最终地图长度。
```

每个锚点建议连续发布：

```text
频率: 5 Hz ~ 20 Hz
持续: 0.5 s ~ 2 s
```

不建议只发布一帧。

### 4.8 发布脚本设计

建议实现一个独立 ROS 节点或脚本，不修改 Cartographer 代码。

输入：

```text
x_anchor
```

内部参数：

```text
lat0
lon0
alt0
publish_rate
publish_duration
```

输出：

```text
sensor_msgs/NavSatFix -> /fix
```

伪代码：

```python
import math

def xy_to_navsat(x, y, lat0_deg, lon0_deg):
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat0_deg))

    lat = lat0_deg + y / meters_per_deg_lat
    lon = lon0_deg + x / meters_per_deg_lon
    return lat, lon

def publish_anchor_fix(x):
    lat, lon = xy_to_navsat(x, 0.0, lat0, lon0)
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = 0.0
    publish_for_duration(msg, duration=1.0, rate=10.0)
```

触发方式可选：

```text
方式 A：命令行参数触发
  rosrun anchor_tools publish_anchor_fix.py _x:=10

方式 B：键盘交互触发
  输入 10 后发布 x=10
  输入 20 后发布 x=20

方式 C：订阅 std_msgs/Float64
  外部人工按钮或上位机发布 x 值
  节点转换为 /fix
```

推荐方式：

```text
人工到达锚点后，由操作员输入锚点编号触发发布。
```

理由：

- 避免使用机器人自身里程计判断是否到达锚点。
- 保证外部约束来源独立。
- 避免将同源里程计误差重复输入后端。

### 4.9 验证步骤

不建议首次直接跑完整 100 m。推荐分阶段验证。

#### 第一阶段：起点静态验证

```text
机器人放在 anchor_0。
启动 Cartographer。
持续发布 x = 0 的 fake NavSatFix。
确认系统无 NavSatFix 或 TF 报错。
```

#### 第二阶段：10 m 验证

```text
机器人从 x = 0 移动到 x = 10。
到点后发布 x = 10 的 fake NavSatFix。
等待一次 pose graph 优化。
检查轨迹终点是否接近 10 m。
```

#### 第三阶段：30 m 验证

```text
依次发布 x = 0, 10, 20, 30。
检查地图长度是否接近 30 m。
检查地图是否出现明显扭曲。
```

#### 第四阶段：100 m 完整验证

```text
每 10 m 发布一次 fake NavSatFix。
最终检查隧道长度是否接近 100 m。
```

### 4.10 调参建议

如果地图仍然压缩：

```text
提高 fixed_frame_pose_translation_weight。
降低 optimize_every_n_nodes，使后端优化更频繁。
检查 /fix 是否实际进入 Cartographer。
检查 /fix 时间戳是否为到达锚点时刻。
```

如果地图被强行拉扭：

```text
检查 fake GPS 经纬度方向是否与隧道 +x 一致。
检查起点 /fix 是否为 x=0。
检查机器人初始 yaw 是否确实为 0。
降低 fixed_frame_pose_translation_weight。
减少每个锚点重复发布次数。
```

如果优化后轨迹整体旋转：

```text
检查第一个 /fix 的参考原点。
检查 Cartographer map 坐标系与人工经纬度 east/north 坐标系的方向关系。
确认启动前机器人朝向与隧道方向一致。
```

### 4.11 fixed-frame/GPS 方案优缺点

优点：

- 不需要修改 Cartographer 代码。
- 直接表达“当前机器人 global 位置已知”的语义。
- 与人工地面锚点的工程含义一致。
- 可以使用 Cartographer 已有 fixed-frame pose 后端残差。
- 实施成本低，只需配置和发布 ROS 话题。

缺点：

- 需要将局部米制坐标转换为虚拟经纬度。
- 只通过 `NavSatFix` 输入时，主要约束位置，不直接约束 yaw。
- 若发布时刻不准确，会把错误位置强行加入后端。
- 需要保证第一个 fake GPS 与建图原点严格一致。

---

## 5. 两种方案对比

| 对比项 | 虚拟 Landmark 方案 | 人工 fixed-frame/GPS 方案 |
|---|---|---|
| 核心语义 | 机器人观测到固定地标 | 机器人当前位置在固定坐标系下已知 |
| 是否符合地面锚点 | 符合 | 符合 |
| 不改代码可完整实现 | 否 | 是 |
| ROS 标准入口 | `/landmark` | `/fix` |
| 是否需要设置 global pose | 需要 | 不需要单独设置 landmark |
| 是否需要 frozen | 需要 | 不涉及 |
| 对位置约束能力 | 强，前提是 frozen | 强 |
| 对 yaw 约束能力 | 可约束 yaw | 默认不强约束 yaw |
| 实施复杂度 | 中到高 | 低 |
| 推荐程度 | 允许改代码时推荐 | 不改代码时优先推荐 |

---

## 6. 推荐实施路线

### 6.1 不修改代码的推荐路线

采用人工 fixed-frame/GPS 方案：

```text
1. lua 中设置 use_nav_sat = true。
2. 设置 fixed_frame_pose_sampling_ratio = 1。
3. 设置 fixed_frame_pose_translation_weight = 1e3 起步。
4. 定义虚拟经纬度原点 lat0/lon0。
5. 将 x = 0, 10, ..., 100 m 转换成 fake NavSatFix。
6. 每到一个地面锚点，人工触发发布对应 /fix。
7. 分阶段验证 10 m、30 m、100 m 效果。
```

该路线不需要修改 Cartographer 或 Cartographer ROS。

### 6.2 允许修改代码的推荐路线

若需要严格以 landmark 形式实现，推荐：

```text
1. 在 Cartographer ROS 中增加固定 landmark 配置加载或 SetLandmarkPose ROS service。
2. 启动时设置 anchor_0...anchor_100 的 global pose。
3. 设置 frozen = true。
4. lua 中设置 use_landmarks = true。
5. 每到一个锚点，发布 /landmark，transform 为 Identity。
6. 根据效果调 translation_weight 和 rotation_weight。
```

该路线语义清晰，但需要维护自定义 Cartographer ROS 代码。

---

## 7. 锚点间距选择分析

### 7.1 间距问题的本质

锚点间距决定了外部绝对约束在轨迹上的空间采样密度。间距越小，后端获得的绝对位置约束越密集；间距越大，两次绝对约束之间主要依赖激光 scan matching、里程计、IMU 和子图-节点约束维持轨迹尺度。

在长直隧道退化场景中，轴向误差通常会随行驶距离累计。因此，锚点间距的核心作用是限制“无绝对约束区间”的最大长度：

```text
锚点间距越小：
  单段无绝对约束距离越短；
  每段内可累计的轴向压缩误差越小；
  后端更容易恢复真实尺度。

锚点间距越大：
  单段无绝对约束距离越长；
  中间轨迹仍可能发生明显压缩；
  只有靠近锚点处被拉回，局部地图可能产生非均匀形变。
```

因此，锚点间距不是越小越绝对好，而是在以下因素之间取折中：

```text
建图精度需求
隧道退化严重程度
里程计可靠性
人工打点和触发成本
后端优化稳定性
```

### 7.2 锚点过密的影响

例如每 5 m 设置一个锚点：

```text
anchor_0, anchor_5, anchor_10, ..., anchor_100
```

优点：

- 绝对位置约束密集，轴向尺度最不容易漂移。
- 每段只允许 5 m 内自由累计误差，压缩问题会被更早、更频繁地纠正。
- 对极端退化场景更稳健。
- 建图结果通常更接近真实几何长度。

缺点：

- 人工打点工作量翻倍。
- 建图时人工触发频率增加，操作复杂度上升。
- 如果触发时间存在延迟或误触发，错误约束数量也会增加。
- 强约束过密时，若某个锚点观测时刻不准，局部轨迹可能被明显拉扯。
- 优化问题中的外部约束数量增加，但 100 m 场景中约束数量仍然很少，计算量通常不是主要问题。

结论：

```text
5 m 间距通常会提高约束效果，但未必带来成比例收益。
在人工成本较高的情况下，只有当 10 m 间距仍不能满足精度要求时，才建议缩小到 5 m。
```

### 7.3 锚点过稀的影响

例如每 50 m 设置一个锚点：

```text
anchor_0, anchor_50, anchor_100
```

优点：

- 人工打点和触发工作量很低。
- 数据管理简单。
- 约束数量少，配置和操作成本低。

缺点：

- 0~50 m、50~100 m 两个区间内仍有很长距离缺少绝对位置约束。
- 如果隧道退化严重，50 m 区间内可能已经发生明显压缩。
- 后端优化只能在锚点处强行满足绝对位置，中间局部地图可能被非线性拉伸或挤压。
- 若原始建图从 100 m 压缩到 40 m，说明退化非常严重，50 m 间距大概率不足。

结论：

```text
50 m 间距只能提供粗略全局尺度校正。
对于已观察到 100 m 被压缩成 40 m 的严重退化场景，50 m 锚点间距通常偏稀，不建议作为首选。
```

### 7.4 不同间距的工程效果预期

以下为 100 m 直线隧道场景的经验性判断：

| 锚点间距 | 锚点数量 | 人工成本 | 约束效果 | 适用情况 |
|---|---:|---|---|---|
| 5 m | 21 个 | 高 | 很强 | 退化极严重、精度要求高、允许较多人工操作 |
| 10 m | 11 个 | 中 | 强 | 推荐初始方案，通常能显著抑制压缩 |
| 20 m | 6 个 | 较低 | 中等 | 退化中等、里程计较可靠、精度要求一般 |
| 25 m | 5 个 | 较低 | 中等偏弱 | 可作为低成本折中方案 |
| 50 m | 3 个 | 低 | 弱到中等 | 只适合粗略尺度修正，不适合严重压缩场景 |

对于当前问题描述中的压缩程度：

```text
真实 100 m -> 建图约 40 m
```

该退化程度较严重，推荐优先使用：

```text
10 m 间距
```

如果 10 m 间距后仍无法满足精度要求，再缩小到：

```text
5 m 间距
```

如果 10 m 间距已经能够满足最终地图长度和局部形状要求，则没有必要使用 5 m 间距。

### 7.5 间距与权重的关系

锚点间距和约束权重是两个不同维度：

```text
锚点间距：
  决定绝对约束出现的空间频率。

约束权重：
  决定每个绝对约束在优化中的可信程度。
```

不能简单地用“很高权重的稀疏锚点”完全替代“合理密度的锚点”。例如只在 0 m 和 100 m 设置强约束：

```text
anchor_0
anchor_100
```

后端可以保证起点和终点大体正确，但中间 100 m 的轨迹如何分布仍由激光、里程计和子图约束决定。若中间退化严重，可能出现：

```text
前半段压缩、后半段拉伸；
局部子图间距不均匀；
墙体纹理或障碍物位置沿 x 方向变形。
```

因此，强约束并不意味着锚点可以任意稀疏。锚点间距应保证每个无绝对约束区间内的退化误差仍处于可接受范围。

### 7.6 推荐选择流程

推荐采用逐步验证方式确定间距，而不是一次性决定最终密度。

#### 第一轮：10 m 间距验证

```text
anchor_0, anchor_10, ..., anchor_100
```

验证指标：

- 最终隧道长度是否接近 100 m。
- 每 10 m 锚点附近轨迹是否接近对应 x 值。
- 地图局部是否出现拉扯、折弯、非均匀形变。
- 闭环或全局优化后轨迹是否稳定。

若结果满足工程要求，则采用 10 m 间距。

#### 第二轮：局部加密验证

如果 10 m 间距整体有效，但某些区间仍存在明显压缩，可只在问题区间加密，而不是全线 5 m。

示例：

```text
常规区间:
  每 10 m 一个锚点

严重退化区间 40~60 m:
  增加 anchor_45, anchor_55
```

这种方式可以降低人工成本，同时针对性增强约束。

#### 第三轮：5 m 间距验证

如果 10 m 间距无法满足整体尺度和局部形状要求，再全线使用 5 m 间距。

适用条件：

```text
退化极严重；
里程计可信度较低；
地图尺度精度要求很高；
人工打点和触发成本可以接受。
```

#### 第四轮：稀疏间距对比

如果目标只是粗略恢复总长度，可测试 20 m 或 25 m 间距。但对于 100 m 被压缩到 40 m 的场景，不建议直接使用 50 m 间距作为主方案。

### 7.7 推荐结论

针对本文的 100 m 长直隧道场景，推荐锚点间距策略如下：

```text
默认推荐:
  10 m 间距。

高精度或严重退化:
  5 m 间距，或在退化严重区间局部加密到 5 m。

低人工成本折中:
  20 m 间距可测试，但需要验证中间轨迹是否仍压缩。

不推荐作为首选:
  50 m 间距。
```

工程上更推荐采用：

```text
先 10 m 全线验证；
再根据误差分布局部加密；
最后才考虑全线 5 m。
```

该策略通常能在建图效果和人工工作量之间取得较合理平衡。

---

## 8. 风险与注意事项

### 8.1 时间戳必须准确

无论使用 landmark 还是 fake GPS，观测时间都必须对应机器人实际位于锚点的时刻。如果到达锚点后延迟数秒才发布，并且消息时间戳使用延迟后的当前时间，则后端会把错误时刻的机器人位姿约束到锚点，导致轨迹被拉扭。

建议：

```text
人工触发时机器人短暂停在锚点。
连续发布 0.5 s ~ 2 s。
使用当前 ROS 时间作为 header.stamp。
```

### 8.2 不建议用里程计自动触发锚点

如果使用轮式里程计积分判断“已到 10 m”并自动发布强约束，则锚点观测与里程计同源，独立性较弱。更推荐：

```text
由人工在地面精密打点位置触发。
```

### 8.3 权重不宜无限大

虽然锚点位置绝对准确，但优化问题仍包含激光、里程计、submap-node 等多类约束。过高权重可能导致数值条件变差或局部地图扭曲。

建议逐步调参：

```text
1e3 -> 1e4 -> 1e5
```

### 8.4 起点对齐是必要前提

本文方案依赖：

```text
建图起点 = global 原点
机器人初始 yaw = 隧道轴线 yaw
```

如果初始 yaw 存在误差，fake GPS 会把轨迹拉到预定义 x 轴上，可能产生地图整体扭曲。

---

## 9. 结论

针对长 100 m 笔直隧道中的 Cartographer 2D 建图压缩问题，人工地面锚点可以作为有效的外部绝对约束。

在不修改 Cartographer 代码的前提下，推荐使用人工 fixed-frame/GPS 方案。该方案通过发布 fake `sensor_msgs/NavSatFix`，将每个 10 m 地面锚点表达为机器人在 fixed frame 下的绝对位置观测，能够直接约束轨迹轴向尺度，实施成本低，且与现有 Cartographer ROS 接口兼容。

虚拟 landmark 方案在理论上同样成立，并且语义上与“固定地面锚点”一致。但完整实现要求预先设置每个 landmark 的 global pose 并将其 frozen。普通 Cartographer ROS 的 `/landmark` 话题只能发布观测，不能设置 frozen landmark，因此若选择该方案，需要扩展 Cartographer ROS 接口或启动配置逻辑。

最终推荐：

```text
短期工程落地：
  使用 fake NavSatFix / fixed-frame pose。

长期规范实现：
  增加 fixed frozen landmark 配置或 ROS service，
  采用虚拟 landmark 作为显式锚点约束。
```
