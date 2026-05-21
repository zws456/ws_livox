# Nav2 导航链路中 2D 栅格地图的原理与局限

> 本文档说明当前导航链路中 2D 栅格地图 (`/projected_map`) 的生成机制，以及它在动态避障方面的局限性。

---

## 1. 2D 栅格地图能感知动态障碍物吗？

**结论：不能直接感知，存在明显滞后。**

当前 `nav2_params.yaml` 中，local_costmap 和 global_costmap 均只配置了：

```yaml
plugins: ["static_layer", "inflation_layer"]
```

这意味着：

- **没有 `obstacle_layer` 或 `voxel_layer`**：Nav2 不会直接订阅原始点云或激光扫描来做实时障碍物检测。
- 代价地图仅将 `/projected_map` 视为一张**准静态地图**来使用。
- 当动态障碍物（如行人）突然出现时，Nav2 的 local_costmap **无法立刻响应**，原因包括：
  1. OctoMap Server 需要接收若干帧点云后，才更新其内部八叉树；
  2. 然后 OctoMap 才重新投影并发布 `/projected_map`；
  3. 该投影更新的频率通常有限（典型值 1~5 Hz）。

> **因此，当前链路不具备实时动态避障能力。**

---

## 2. OctoMap Server 的建图机制

### 2.1 数据来源

见 `src/FAST_LIO_ROS2/launch/realtime_mapping.launch.py`：

```python
octomap_server = Node(
    package='octomap_server',
    executable='octomap_server_node',
    remappings=[
        # 订阅 /cloud_registered：当前帧点云，数据量小
        ('cloud_in', '/cloud_registered'),
    ],
    parameters=[{
        'resolution': 0.05,                # 八叉树分辨率 5cm
        'frame_id': 'camera_init',         # 地图坐标系
        'base_frame_id': 'body',           # 机器人底盘坐标系
        'sensor_model.max_range': 30.0,    # 最大感知范围
        'pointcloud_min_z': 0.2,           # 过滤地面
        'pointcloud_max_z': 1.5,           # 过滤天花板
        'occupancy_min_z': 0.2,            # 投影 2D 时的最低高度
        'occupancy_max_z': 1.5,            # 投影 2D 时的最高高度
        'filter_ground': False,
    }]
)
```

注意：这里订阅的是 **`/cloud_registered`**（FAST_LIO 输出的当前帧配准点云），而不是 `/Laser_map`（全局累积大地图）。这样设计是为了降低传输带宽和 OctoMap 的输入处理压力。

### 2.2 内部原理

OctoMap Server 的工作流程如下：

```
每一帧新点云 (/cloud_registered)
        │
        ▼
┌──────────────────┐
│  octomap_server  │  ← 内部维护一个增量累积的 3D OctoMap（八叉树）
│  (3D occupancy)  │    新点云不断融合进去，历史地图数据持续保留
└──────────────────┘
        │
        ▼  周期性地把当前完整的 3D OctoMap
        ▼  沿 Z 轴方向投影压缩
        ▼
   /projected_map  (nav_msgs/OccupancyGrid, 2D)
```

关键结论：

| 问题 | 答案 |
|------|------|
| 是直接拿当前帧 3D 点云压成 2D 吗？ | **不是**。OctoMap 内部维护的是一个**持续累积的 3D 八叉树地图**。 |
| 是累计 3D 后再压吗？ | **是的**。新点云不断融合进历史 3D 地图，然后周期性地整体投影发布 2D 栅格。 |

也就是说：
- **3D 层面**：持续增量建图，地图范围随探索逐渐扩大。
- **2D 层面**：`/projected_map` 只是这个累积 3D 地图的一个**俯视投影快照**。

---

## 3. 当前链路的问题总结

| 问题 | 原因 | 影响 |
|------|------|------|
| 无法实时动态避障 | Costmap 缺少 `obstacle_layer`，仅依赖 OctoMap 投影 | 行人/突然出现物体会撞上或导致规划失败 |
| 障碍物移除滞后 | OctoMap 需要通过空闲射线更新才能标记为 free，再投影 2D | 障碍物移走后，地图仍显示为 occupied，路径可能被阻塞 |
| 地图更新频率低 | `/projected_map` 发布频率远低于点云帧率 | 代价地图对变化的响应迟钝 |
| 高度信息丢失 | 3D 投影成 2D 时，`occupancy_min_z` ~ `occupancy_max_z` 之外的信息全部丢弃 | 低空或高处障碍物无法区分 |

---

## 4. 后续优化方向（如需改进）

- **给 `local_costmap` 增加 `obstacle_layer`**：直接订阅 `/cloud_registered` 或 `/Laser_map`，让 Nav2 能感知原始点云中的实时障碍物。
- **调整 OctoMap 参数**：缩小 `sensor_model.max_range`、增大投影频率，降低滞后。
- **考虑使用 3D 导航**：如果机器人需要在多层空间（如楼梯、斜坡）运行，2D 投影会丢失关键信息，可评估是否需要 3D 规划器（如 `motion_planner` 或 `elevation_mapping`）。

---

*文档生成时间：2026-04-23*
*对应配置版本：`src/nav2_bringup_config/config/nav2_params.yaml`、`src/FAST_LIO_ROS2/launch/realtime_mapping.launch.py`*
