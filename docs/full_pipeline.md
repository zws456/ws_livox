# FAST_LIO + AMCL 完整链路

## 一、系统架构

```
建图阶段
========
Livox MID360 ──→ FAST_LIO ──→ /Laser_map (3D点云地图)
         │           │
         └──→ IMU ───┘           │
                                  ↓
                           OctoMap Server
                                  ↓
                           /projected_map (2D栅格地图)
                                  ↓
                           map_saver_cli 保存
                                  ↓
                           my_map.pgm + my_map.yaml

导航阶段（实时模式，无地图）
==========================
Livox MID360 ──→ FAST_LIO ──→ /Odometry (里程计)
         │           │
         └──→ IMU ───┘           │
                                  ├──→ /cloud_registered_body ──→ pointcloud_to_laserscan ──→ /scan
                                  │                                                            │
                                  └────────────────────────────────────────────────────────────┘
                                                                                               ↓
                                                                                         Nav2 导航
                                                                               订阅 /projected_map 实时地图
                                                                                               ↓
                                                                                         RViz（最后启动）

导航阶段（AMCL 静态地图模式）
============================
Livox MID360 ──→ FAST_LIO ──→ /Odometry (里程计)
         │           │
         └──→ IMU ───┘           │
                                  ├──→ /cloud_registered_body ──→ pointcloud_to_laserscan ──→ /scan ──┐
                                  │                                                                    │
                                  └────────────────────────────────────────────────────────────────────┤
                                                                                                       ↓
map_server ──→ /map (静态地图) ──┬──→ AMCL ──→ 发布 TF: map → camera_init                             │
                                │                                                                     │
                                └──→ Nav2 代价地图                                                     │
                                                                                                       ↓
                                                                                              组合定位 + 导航
                                                                                         TF树: map → camera_init → body
                                                                                                       ↓
                                                                                              RViz（最后启动，自动加载 /map）
```

## 二、坐标系关系

```
map (全局固定，永不重置)          ← 路标点基于这里
  └── [AMCL] → camera_init (每次重启重置)
         └── [FAST_LIO] → body (机器人本体)
```

- **FAST_LIO**：发布 `camera_init → body`，只管局部里程计
- **AMCL**：发布 `map → camera_init`，解决全局定位
- **组合后**：`map → camera_init → body`，body 的全局位置就准了

## 三、文件清单

| 文件 | 作用 |
|------|------|
| `src/nav2_bringup_config/config/pointcloud_to_laserscan.yaml` | 3D→2D 转换参数 |
| `src/nav2_bringup_config/config/nav2_params.yaml` | Nav2 全参数（AMCL/代价地图/控制器） |
| `src/nav2_bringup_config/launch/nav2_bringup.launch.py` | 纯 Nav2 启动（被导航 launch 复用） |
| `src/nav2_bringup_config/launch/mapping_with_save.launch.py` | **建图阶段**启动 |
| `src/nav2_bringup_config/launch/navigation_with_amcl.launch.py` | **导航阶段**启动 |
| `src/nav2_bringup_config/scripts/waypoint_navigator.py` | 特定点导航节点 |
| `src/nav2_bringup_config/waypoints/waypoints.yaml` | 固定点坐标 |


## 四、使用指令

### 阶段一：建图 + 保存地图

**终端 1：启动建图**
```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 launch nav2_bringup_config mapping_with_save.launch.py
```
- 会自动弹出 RViz2
- 带着雷达走一圈，左侧 ProjectedMap 会显示 2D 栅格地图

**终端 2：保存 2D 地图**
```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -t /projected_map -f ~/my_map
```
- 生成 `~/my_map.pgm` 和 `~/my_map.yaml`

**终端 3：标记固定点坐标**
```bash
ros2 topic echo /clicked_point
```
- RViz2 顶部工具栏点击 **"Publish Point"**
- 点击目标位置（如饮水机）
- 终端记录 x, y 坐标
- 填入 `src/nav2_bringup_config/waypoints/waypoints.yaml`

```yaml
vending_machine:
  x: 5.23
  y: 3.14
  yaw: 0.0
```

---

### 阶段二：导航（两种模式）

#### 模式 A：实时模式（无静态地图，无 AMCL）

适合：临时测试、不依赖保存的地图
```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 launch nav2_bringup_config navigation_with_amcl.launch.py
```
- 不给 `map` 参数
- 包含：雷达 + FAST_LIO + pointcloud_to_laserscan + Nav2
- `camera_init` 原点重置，没有全局定位
- RViz 最后启动

#### 模式 B：AMCL 静态地图模式（推荐）

适合：正式运行、需要全局定位、去固定点
```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 launch nav2_bringup_config navigation_with_amcl.launch.py map:=/home/zws/new_map.yaml
```
- 包含：**雷达 + FAST_LIO + pointcloud_to_laserscan + map_server + AMCL + Nav2 + RViz**
- 一条指令全部启动，**不需要**单独开雷达或 FAST_LIO
- RViz 在所有节点就绪后最后启动，地图自动显示，无需手动刷新

**初始定位（重要）：**
```bash
# 方式 1：RViz2 中点击 "2D Pose Estimate"，在地图上拖动
# 方式 2：命令行
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.068]
  }
}" --once
```

> **注意**：如果地图未显示，检查 RViz 左侧 `Map` 插件的 Topic 是否为 `/map`（`fastlio.rviz` 已默认改为 `/map`）。若仍不显示，执行：
> ```bash
> ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '/home/zws/new_map.yaml'}"
> ```

---

### 导航方式（两种互不干扰）

#### 方式 1：正常导航（想去哪去哪）
```bash
# RViz2 点击 "Nav2 Goal"
# 或命令行
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 2.0, z: 0.0},
    orientation: {z: 0.0, w: 1.0}
  }
}" --once
```

#### 方式 2：特定点导航（饮水机、充电站等）

**启动特定点导航节点：**
```bash
ros2 run nav2_bringup_config waypoint_navigator.py
```

**发送指令去指定点：**
```bash
ros2 topic pub /goto_waypoint std_msgs/msg/String "data: vending_machine" --once
```

**查看导航状态：**
```bash
ros2 topic echo /waypoint_status
```

---

### 辅助检查命令

| 检查项 | 命令 |
|--------|------|
| `/scan` 是否正常 | `ros2 topic hz /scan` |
| `/map` 是否正常 | `ros2 topic hz /map` |
| TF 树 | `ros2 run tf2_tools view_frames` |
| 查看路标点 | `cat src/nav2_bringup_config/waypoints/waypoints.yaml` |

