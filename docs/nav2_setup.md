# Nav2 路径规划接入指南

> 基于 FAST_LIO（实时里程计 + 点云地图）+ OctoMap Server（实时 2D 栅格地图）的 Nav2 导航配置说明。

---

## 1. 前置条件

- FAST_LIO 实时建图已跑通：`ros2 launch fast_lio realtime_mapping.launch.py`
- OctoMap Server 已随上述 launch 启动，实时发布 2D 栅格地图
- 确认以下话题正常输出：
  - `/projected_map` —— OctoMap 发布的 2D 地图
  - `/Odometry` —— FAST_LIO 发布的高精度里程计
  - `camera_init` → `base_link`（或 `body`）的 TF 变换

> **检验方法见下文 [2.1 前置条件检验](#21-前置条件检验)**

---

### 2.1 前置条件检验

在继续之前，先启动你的实时建图：

```bash
ros2 launch fast_lio realtime_mapping.launch.py
```

然后用以下命令逐一检验：

#### ① 检验 `/projected_map`（2D 地图）

```bash
# 查看话题是否存在
ros2 topic list | grep projected_map

# 查看发布频率（正常应该 > 0 Hz）
ros2 topic hz /projected_map

# 查看地图内容（按 Ctrl+C 退出）
ros2 topic echo /projected_map
```

**正常表现**：`ros2 topic hz` 能显示出稳定的频率（比如 1~5 Hz）；`ros2 topic echo` 能看到 `nav_msgs/OccupancyGrid` 格式的数据，其中 `info.resolution` 应该是 0.05，`data` 数组里有非 `-1` 的值。


#### ② 检验 `/Odometry`（里程计）

```bash
# 查看话题是否存在
ros2 topic list | grep Odometry

# 查看发布频率（正常应该 >= 10 Hz）
ros2 topic hz /Odometry

# 查看完整内容（position 和 orientation 应该随雷达移动而变化）
ros2 topic echo /Odometry

# 或者只过滤 position 部分看
ros2 topic echo /Odometry | grep -A 3 position
```

**正常表现**：
- `ros2 topic hz` 显示 **10~100 Hz** 的稳定频率。
- 机器人**静止**时：`x`、`y`、`z` 数值基本不变（可能会有微小漂移，这是正常的）。
- 机器人**移动**时：`x`、`y` 数值会随移动方向和距离实时变化。比如向前直走 1 米，`x` 应该增加约 1.0（取决于坐标系朝向）。

**快速验证技巧**：
```bash
# 只看 x, y 值，方便观察变化
ros2 topic echo /Odometry/pose/pose/position/x
ros2 topic echo /Odometry/pose/pose/position/y
```

> **⚠️ 注意**：`/Odometry` 是 FAST_LIO 在 **世界坐标系 `camera_init`** 下的**绝对位姿**。意思是：FAST_LIO 启动瞬间，雷达所在位置被固定为原点 `(0, 0, 0)`，之后 `/Odometry` 里的 `x`、`y`、`z` 都是相对于这个原点的坐标。雷达离原点越远，数值越大；拿回原点附近，数值回到 0。它不会自动清零，重启 FAST_LIO 才会重置原点。

#### ③ 检验 TF 变换（`camera_init` → `base_link`）

```bash
# 查看 TF 树整体结构（会生成 frames.pdf 在当前目录）
ros2 run tf2_tools view_frames

# 实时查询两个坐标系之间的变换关系
ros2 run tf2_ros tf2_echo camera_init base_link
```

**正常表现**：
- `view_frames` 生成的 PDF 里能看到 `camera_init` → `base_link`（或 `body`）的链，没有断开的红色节点。
- `tf2_echo` 会持续输出 `Translation` 和 `Rotation` 数值，且机器人在运动时这些数值会实时变化。

> **注意**：如果你的机器人底盘坐标系名称不是 `base_link`（FAST_LIO 里可能叫 `body`），后续 `nav2_params.yaml` 里的 `robot_base_frame` 要改成实际名称。

---

## 3. 安装 Nav2

```bash
sudo apt update
sudo apt install ros-humble-nav2-bringup ros-humble-nav2-util ros-humble-nav2-simple-commander
```

---

## 3. 地图话题对接

OctoMap Server 默认发布 **`/projected_map`**，但 Nav2 默认订阅 **`/map`**。

| 方案 | 做法 | 推荐度 |
|------|------|--------|
| A | 启动 Nav2 时将 `/map` remap 到 `/projected_map` | ⭐ 推荐 |
| B | 修改 `realtime_mapping.launch.py`，让 OctoMap 直接发布到 `/map` | 备选 |

**方案 A 示例**（在 Nav2 launch 中）：
```python
remappings=[
    ('/map', '/projected_map'),
]
```

---

## 4. 创建 Nav2 参数文件

新建 `nav2_params.yaml`，以下是关键配置项：

### 4.1 坐标系与话题

```yaml
amcl:
  ros__parameters:
    # 建议关闭 AMCL，FAST_LIO 的里程计精度足够替代定位
    use_sim_time: false

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: camera_init      # 与 FAST_LIO 世界坐标系一致
      robot_base_frame: base_link
      update_frequency: 1.0
      publish_frequency: 1.0
      # 代价地图数据源：订阅 OctoMap 的 2D 地图
      # 实际配置请参考 nav2_bringup 默认模板调整

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: camera_init
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      # 如果 /projected_map 更新慢，可额外订阅 /cloud_registered 作为 obstacle_layer 输入
```

### 4.2 规划器

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5打开 RViz2 后，左下角 Add → 添加以下 Display：
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # DWB 参数可根据机器人实际动力学调整
```

> **完整模板参考**：`/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`，复制一份在此基础上修改。

---

## 5. 创建 Nav2 Launch 文件

新建 `nav2_bringup.launch.py`：

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': '/path/to/your/nav2_params.yaml',
            'use_composition': 'False',   # 调试阶段建议 False，方便看日志
            'map': '',                     # 不加载静态地图，实时订阅
        }.items()
    )
    
    return LaunchDescription([
        nav2_launch,
    ])
```

如果需要 remap 地图话题：
```python
from launch_ros.actions import Node

# 或者通过 Node 的 remappings 参数处理
```

---

## 6. 启动顺序

### 终端 1：实时建图（已有）
```bash
ros2 launch fast_lio realtime_mapping.launch.py
```

### 终端 2：启动 Nav2
```bash
source install/setup.bash
ros2 launch <你的配置包名> nav2_bringup.launch.py
```

### 终端 3：检查环境
```bash
# 查看地图话题
ros2 topic list | grep map
ros2 topic hz /projected_map

# 检查 TF 树是否连通（camera_init -> base_link）
ros2 run tf2_tools view_frames

# 查看当前机器人位姿
ros2 topic echo /Odometry
```

---

## 7. 发送导航目标点

### 7.1 RViz2（图形化）

```bash
ros2 run rviz2 rviz2
```

- 左侧面板添加 **Nav2 Goal** 工具
- 在地图上点击目标位置并拖动指定朝向

### 7.2 命令行

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'camera_init'},
  pose: {
    position: {x: 1.0, y: 2.0, z: 0.0},
    orientation: {z: 0.0, w: 1.0}
  }
}" --once
```

### 7.3 Python 脚本（Simple Commander）

适合后续开发自动巡航、多点导航等逻辑：

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

nav = BasicNavigator()
# ... 设置初始位姿、发送目标点、等待结果
```

---

## 8. 常见问题排查

| 现象 | 排查方向 |
|------|----------|
| Nav2 报 `No map received` | 检查 `/projected_map` 是否发布；检查代价地图配置的话题名是否匹配 |
| TF 报错 `camera_init` 不存在 | 确认 FAST_LIO 已启动；确认 `nav2_params.yaml` 中的 `global_frame` 写的是 `camera_init` |
| 机器人收到目标但不动 | 检查 `/cmd_vel` 是否有输出；检查 DWB 的 `max_vel_x` 等参数是否为 0 或过小；检查代价地图是否空白 |
| 代价地图空白 | **先检查 RViz2 的 Color Scheme 是否为 `costmap`**（默认 `map` 会显示为透明）；再排查 OctoMap 的 `/projected_map` 更新频率是否过低 |
| 定位漂移导致导航失败 | 确认是否关闭了 AMCL；FAST_LIO 的里程计在长距离后可能累积漂移，大场景建议配合回环检测或先建图后导航 |

---

## 9. 后续优化方向

- [ ] 将 `local_costmap` 数据源从纯地图改为**地图 + 实时点云**双输入，提升动态避障能力
- [ ] 配置 `behavior_server` 和 `bt_navigator`，自定义行为树（如到达目标后执行特定动作）
- [ ] 使用 `nav2_simple_commander` 编写 Python 脚本，实现多点巡逻、自动返航等功能
- [ ] 大场景下先用 FAST_LIO 建图并保存为 `.pgm` + `.yaml`，再切换为**纯定位模式** + Nav2，降低计算负载
