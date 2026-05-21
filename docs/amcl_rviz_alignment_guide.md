# AMCL 静态地图定位与 RViz 对齐操作指南

> 本文档描述如何在 Nav2 静态地图模式下，使用 RViz 判断 AMCL 定位是否正确，以及如何通过 2D Pose Estimate 手动对齐。

---

## 一、RViz 必备插件清单

启动静态地图模式后，在 RViz 中添加以下显示插件：

| 插件类型 | Topic | QoS Durability | 作用 |
|---------|-------|---------------|------|
| **Map** | `/map` | `Transient Local` | 显示静态栅格地图 |
| **Map** | `/global_costmap/costmap` | `Transient Local` | 显示全局代价地图 |
| **Map** | `/local_costmap/costmap` | `Transient Local` | 显示局部代价地图（实时窗口） |
| **LaserScan** | `/scan` | `Reliable` | 显示 2D 激光扫描线 |
| **TF** | — | — | 显示坐标变换树 |
| **PoseArray**（可选） | `/particlecloud` | `Volatile` | 显示 AMCL 粒子分布 |

**LaserScan 推荐显示参数：**
- **Size (m)**: `0.05` ~ `0.1`
- **Color**: 固定颜色（如红色 `#ff0000`）
- **Style**: `Flat Squares`（最显眼）
- **Buffer Length**: `1`

**Fixed Frame 必须设为 `map`。**

---

## 二、如何判断 AMCL 是否对准

### 方法 1：观察 LaserScan 与 Map 的重合度（最直观）

在 RViz 中同时观察 `/map`（静态地图）和 `/scan`（激光线）：

| 状态 | 视觉特征 |
|------|---------|
| ✅ **对准了** | `/scan` 的白色/红色激光点形成清晰的"轮廓线"，刚好贴在 `/map` 黑色障碍物（墙壁、柱子）的边缘上，没有穿墙、没有飘在空旷区域 |
| ❌ **没对准** | `/scan` 的激光点散乱分布，穿过地图墙壁，或聚集在地图空白区域，与黑色障碍物明显错开 |

**示意图：**

```
对准了：                    没对准：
    ┃●                          ┃   ●
    ┃●                          ┃  ● ●
    ┃●                          ┃ ●   ●
    ┃●                         ●      ●
    ┃●                        ●        ●
```

### 方法 2：查看 AMCL 位姿协方差（命令行）

```bash
source /home/zws/ws_livox/install/setup.bash
ros2 topic echo /amcl_pose --once
```

观察输出中的 `covariance` 数组：

```yaml
covariance:
  - ...
  - ...
  - ...
  - ...
  - ...
  - 0.05    # ← 最后一个数字是 yaw（方向）的不确定性
```

| 协方差值 | 状态 |
|---------|------|
| `< 0.1` | ✅ 收敛良好，方向确定 |
| `0.1 ~ 1.0` | ⚠️ 大致收敛，但仍有漂移 |
| `> 1.0` | ❌ 未收敛，方向高度不确定 |

**如果 yaw 协方差大于 1.0，说明 AMCL 还在猜测，需要手动对齐。**

### 方法 3：观察 TF 是否稳定

添加 **TF** 插件，观察 `map → camera_init → body` 的坐标轴：

- **稳定**：机器人模型在地图上静止不动（如果你没动机器人）
- **不稳定**：机器人模型每隔几秒突然跳到新位置，或持续漂移

---

## 三、2D Pose Estimate 手动对齐操作

当 AMCL 自动收敛失败（激光线与地图不重合、协方差很大、TF 跳变）时，使用此方法手动纠正。

### 操作步骤

1. **确认 RViz 设置正确**
   - Fixed Frame = `map`
   - 显示 `/map` 和 `/scan`
   - LaserScan 参数设置正确（见第一节）

2. **点击 2D Pose Estimate 工具**
   - RViz 顶部工具栏，点击 **"2D Pose Estimate"**（绿色人形图标）

3. **在地图上选择位置**
   - 在 `/map` 显示的区域中，找到与你**当前实际环境匹配**的位置
   - 例如：如果你在走廊中间，就点在地图的走廊中间；如果你在墙角，就点在墙角

4. **拖动设置方向**
   - 按住鼠标左键**拖动**，拉出一个绿色箭头
   - 箭头方向 = 机器人的**实际朝向**
   - **方向比位置更重要**：如果机器人正对着一堵墙，箭头就指向那堵墙

5. **松开鼠标，等待收敛**
   - 松开后，观察 `/scan` 激光点是否开始"贴"到地图墙壁上
   - 等待 3~5 秒，看粒子云是否聚拢
   - 如果激光线仍然散乱，重复步骤 3~4，换一个位置或方向再试

### 关键技巧

- **位置可以粗略**：差 1~2 米没关系，AMCL 会自动微调
- **方向要尽量准**：偏差超过 45 度时，AMCL 很难收敛
- **多试几次**：如果第一次没对准，重新点，逐步修正
- **看激光线而不是点云**：判断标准是 `/scan` 是否压在 `/map` 墙壁上，不要看 `/Laser_map` 或 `/cloud_registered_body`

---

## 四、快速诊断命令

```bash
# 进入工作空间
source /home/zws/ws_livox/install/setup.bash

# 1. 查看 AMCL 当前位姿
ros2 topic echo /amcl_pose --once

# 2. 查看 AMCL 协方差（判断是否收敛）
ros2 topic echo /amcl_pose --once | grep -A 36 "covariance"

# 3. 查看 /scan 数据是否正常
ros2 topic info /scan

# 4. 查看 /map 是否正常发布
ros2 topic info /map

# 5. 查看 TF 变换
ros2 run tf2_ros tf2_echo map body

# 6. 查看 AMCL 和 map_server 生命周期状态
ros2 lifecycle list /amcl
ros2 lifecycle list /map_server
```

---

## 五、常见问题

### Q1：为什么 `/Laser_map` 和 `/map` 不重合？

**A**：`/Laser_map` 是 FAST_LIO 内部维护的全局 3D 点云地图，和导航无关。之前手动对齐错误时，它已被污染，视觉上会很乱。

**处理**：从 RViz 中删除 `/Laser_map` 显示，不要管它。导航只依赖 `/scan` + `/map`。

### Q2：为什么 `/cloud_registered_body`（实时点云）和 `/map` 看起来不重合？

**A**：实时点云在 `body` 坐标系中，需要通过 `map → camera_init → body` 变换链才能显示在 `map` 坐标系中。如果 AMCL 定位错误，实时点云就会显示在错误位置。

**处理**：先解决 AMCL 定位问题（用 2D Pose Estimate），实时点云自然会对齐。

### Q3：2D Pose Estimate 点了很多次还是不准？

**A**：可能是方向偏差太大。尝试以下步骤：
1. 先确保位置大致正确（在正确的房间/走廊）
2. 重点调整方向：让箭头严格指向机器人当前朝向
3. 每次点完后观察 5 秒，看激光线是否有"聚拢"趋势
4. 如果完全找不到匹配区域，检查 `/scan` 数据是否正常（`ros2 topic echo /scan`）

### Q4：如何保存当前正确位姿，下次启动自动对准？

**A**：如果机器人每次在固定位置启动：

1. 用 2D Pose Estimate 对准后，等待 AMCL 收敛（协方差 < 0.1）
2. 记录位姿：
   ```bash
   ros2 topic echo /amcl_pose --once
   ```
3. 将 `position.x, position.y, position.z` 和四元数 `orientation.z, orientation.w` 填入 `nav2_params.yaml`：
   ```yaml
   initial_pose:
     x: 2.35      # 填入实际 x
     y: -1.08     # 填入实际 y
     z: 0.0
     yaw: 1.57    # 从四元数换算（可用在线转换工具）
   ```
4. 重新编译 `nav2_bringup_config`，下次启动即自动对准。

---

## 六、启动命令参考

```bash
# 1. 进入工作空间
source /home/zws/ws_livox/install/setup.bash

# 2. 启动静态地图模式（雷达 + FAST_LIO + AMCL + Nav2）
ros2 launch nav2_bringup_config navigation_with_amcl.launch.py map:=/home/zws/my_map.yaml

# 3. （如需保存新地图）
ros2 run nav2_map_server map_saver_cli -t /projected_map -f ~/my_map
```
