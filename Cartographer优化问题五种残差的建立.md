# Cartographer优化问题五种残差的建立

cartographer后端优化问题求解在，optimization_problem_2d.cc Solve()函数中。

## 1. 第一种残差 将节点与子图原点在global坐标系下的相对位姿 与 约束 的差值作为残差项

可以理解为对每一条 Constraint，优化器检查“当前估计的子图全局位姿”和“当前估计的节点全局位姿”推出的节点相对子图位姿，是否等于这条约束里记录的节点相对子图位姿；二者不一致的部分就是残差。

```
e_ij = zbar_ij - inverse(T_G_Si) * T_G_Nj
```

“节点”是 TrajectoryNode，也就是一帧经过 local SLAM 处理后的轨迹节点，通常对应一次 scan matching 后的机器人位姿/激光帧位姿。

“子图原点”指的是某个 Submap 自己局部坐标系的原点。每个子图都有一个局部坐标系，子图里维护的栅格地图、插入的点云，都是相对于这个子图坐标系表达的。

### 1.1 子图的 global pose 是怎么来的？

对于第一个子图，第一个子图的 global pose = 第一个子图的 local pose。对于后续新的子图，已知旧子图在 global 下的位置 T_G_Sold；local SLAM 知道旧子图和新子图在 local 坐标系下的位置 T_L_Sold、T_L_Snew；于是用旧子图作为桥，就可以把新子图从 local 坐标系换算到 global 坐标系。

```
T_G_Snew = T_G_Sold * inverse(T_L_Sold) * T_L_Snew
```

### 1.2 节点的 global pose 怎么来的？

节点 global pose 的初始值来自：local SLAM 给出的节点 local pose + 当前子图的 global pose + 当前子图的 local pose。

### 1.3 约束中保存的节点相对于子图的观测位姿是怎么来的？

**两类来源：**

- **INTRA_SUBMAP：**节点插入当前活跃子图时直接生成。这个节点当时就是被 local SLAM 插入到这个子图里的； 所以 local SLAM 给出的节点和子图之间的相对关系，可以作为一条可靠约束。

- **INTER_SUBMAP：**后台 scan matching 匹配出来。INTER_SUBMAP 是节点和“不是它原本插入的子图”之间的约束，通常用于闭环或跨子图匹配。拿节点 j 的点云去和子图 i 的栅格地图做 scan matching；匹配成功后，得到节点 j 在该子图附近的 local 位姿；再换算成“节点 j 相对子图 i”的位姿；这个相对位姿就是闭环约束的观测值。

**两类约束的本质区别：**

- **INTRA_SUBMAP:**

  节点本来就插入了这个子图，约束来自 local SLAM 已有的插入关系，`zbar_ij = inverse(T_L_Si) * T_L_Nj`，一般比较可靠，优化时不加HuberLoss。

- **INTER_SUBMAP:**

  节点不是原本插入这个子图，约束来自后台 scan matching / 回环检测，`zbar_ij = inverse(T_L_Si) * T_L_Nj_matched`，可能误匹配，优化时加 HuberLoss。



## 2. 第二种残差 landmark数据 与 通过2个节点位姿插值出来的相对位姿 的差值作为残差项

### 2.1 什么是 Landmark

在 SLAM 里，landmark 通常指环境中可以被重复识别、并且具有稳定空间位置的“标志物 / 地标 / 特征目标”。landmark 不是普通激光点云里的某个点，也不是 submap，也不是 trajectory node。它是额外输入给后端 pose graph 的一种观测数据，用来约束机器人轨迹和地图。

每次 landmark 数据包含：**time** 这次观测发生的时间。**id** landmark 的唯一 ID，例如 "tag_12"、"reflector_03"、"uwb_anchor_A"。**landmark_to_tracking_transform** 这次观测到的 landmark 和 tracking_frame 之间的相对位姿。**translation_weight / rotation_weight** 这次观测在平移和旋转上的可信度权重。

在 Cartographer 里，一个 landmark node 表示：某个具有唯一 ID 的地标；它可以被机器人在不同时间、多次观测到；每次观测给出该 landmark 与机器人 tracking_frame 的相对位姿；后端优化会估计这个 landmark 在 global/map 坐标系下的位置。

在 Solve() 中，它会把 landmark 也作为优化变量加入 Ceres，优化变量有：前一个节点位姿 **prev_node_pose**，后一个节点位姿 **next_node_pose**，landmark 的 **global rotation**，landmark 的 **global translation**。

为什么用前后两个节点？因为 landmark 观测的时间 observation.time 不一定刚好等于某个 trajectory node 的时间。所以代码会找到观测时间前后的两个节点，在二者之间插值得到观测时刻的 tracking frame 位姿。

## 3. 第三种残差 节点与节点间在global坐标系下的相对坐标变换 与 通过里程计数据插值出的相对坐标变换 的差值作为残差项

