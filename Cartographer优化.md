# Cartographer优化

## 初始化

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