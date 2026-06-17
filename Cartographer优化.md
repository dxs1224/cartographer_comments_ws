# Cartographer优化

## 初始化

### node_main.cc

1. TimerCallback() 5s 执行一次，会将 is_init 置为false。延长触发时间，或者定位未成功时，强制为 true

is_init_ 会绕过 motion filter，让初始化阶段更密集地产生节点和里程计数据，并在 pose graph 中强制按局部窗口尝试添加约束。它能让初始定位更快成功，但前提是初始位姿大致正确；同时它也会让初始化阶段产生更多历史约束。

### pose_graph_2d.cc

#### ComputeConstraint()

1. 降低global_constraint_search_after_n_seconds时间，默认值10s
2. 纯定位时只采用局部窗口约束计算

#### MaybeAddGlobalConstraint()

1. 对整体子图进行约束计算时，增加距离的限制



- 如果想要只调试前端的话，可以把 optimize_every_n_nodes 参数改为0



### constraint_builder_2d.cc

#### ComputeConstraint()

1. 纯定位时是否可以优化？粗匹配、精匹配
2. 室外增大 max_constraint_distance 距离 50 m
3. 初始化阶段 is_init ，将局部地图约束搜索的采样频率 sampling_ratio 加大
4. linear_search_window 搜索窗适当增大
5. 不管有没有定位好，纯定位模式下都只用局部子图约束搜索模式？
5. 算不过来，把分支从7减小





- cartographer纯定位置信度，可以考虑 min_score 或者 global_localization_min_score 分数，同时考虑前几名的最高得分，如果前几名的得分相差不大，说明定位结果不一定可靠，当前点云在地图里有多个相似解，可能是以下场景：

```
长走廊
重复货架
相似房间
对称区域
墙面特征少
激光视野被遮挡
动态障碍较多
```

