# FAST_LIO + AMCL 全局定位与定点导航方案

> 目标：解决 FAST_LIO 每次开机原点重置的问题，实现"保存地图 → 任意位置开机 → 导航到固定坐标"。

---

## 一、方案概述

### 1.1 当前系统的问题

当前链路：`Livox MID360 → FAST_LIO → OctoMap → Nav2`

问题出在 **FAST_LIO 的 `camera_init` 原点每次重启都会重置**。上次开机时标记的"自动售货机坐标 (5.2, 3.1)"是基于旧的 `camera_init`，下次开机后这个坐标就指向了另一个物理位置。

根本原因：**系统只有局部里程计，没有全局定位**。

| 环节 | 当前状态 | 缺失 |
|------|---------|------|
| 里程计 | FAST_LIO 提供 `camera_init → body` | ✅ 有 |
| 全局地图 | OctoMap 实时生成，未保存 | ❌ 缺 |
| 全局定位 | AMCL 未实际运行（无静态地图、无 /scan） | ❌ 缺 |
| 固定坐标系 | 无 `map` 坐标系，只有会重置的 `camera_init` | ❌ 缺 |

### 1.2 本方案思路

在现有系统上叠加 **AMCL 全局定位**，核心链路：

```
建图阶段：FAST_LIO + OctoMap → 保存 2D 地图 (.pgm+.yaml)
         ↓
导航阶段：FAST_LIO (局部里程计) + pointcloud_to_laserscan (3D→2D) + AMCL (全局定位) + Nav2
```

AMCL 会在静态地图上找到机器人的真实位置，发布 `map → camera_init` 的 TF 变换。这样固定点坐标绑定在永不变化的 `map` 坐标系上，和 `camera_init` 是否重置无关。

### 1.3 坐标系关系

```
map (全局固定，由 map_server 加载)          ← 固定点坐标基于这里
  └── [AMCL] → camera_init (FAST_LIO 原点，每次重置)  ← AMCL 算偏移
         └── [FAST_LIO] → body (机器人本体)
```

- **FAST_LIO**：只管 `camera_init → body`，不管全局位置
- **AMCL**：在地图上定位，发布 `map → camera_init`
- **组合后**：`map → camera_init → body`，body 的全局位置就准了

---

## 二、前置条件

- Ubuntu 22.04 + ROS2 Humble
- 已跑通当前系统：`realtime_mapping.launch.py` + `nav2_bringup.launch.py`
- 雷达驱动已配置，能正常输出 `/livox/lidar` 和 `/livox/imu`

---

## 三、安装依赖

```bash
# 3D 点云转 2D 激光扫描
sudo apt install -y ros-humble-pointcloud-to-laserscan

# AMCL（如果之前没装）
sudo apt install -y ros-humble-nav2-amcl

# map_server（保存/加载静态地图）
sudo apt install -y ros-humble-nav2-map-server
```

---

## 四、配置文件修改

### 4.1 新建 pointcloud_to_laserscan 配置

新建文件：`src/nav2_bringup_config/config/pointcloud_to_laserscan.yaml`

```yaml
pointcloud_to_laserscan_node:
  ros__parameters:
    target_frame: "body"              # 输出的 scan 绑定到 body 坐标系
    transform_tolerance: 0.1
    min_height: 0.2                   # 只取 0.2m~1.5m 的点（过滤地面和天花板）
    max_height: 1.5
    angle_min: -3.14159               # -180°
    angle_max: 3.14159                # +180°
    angle_increment: 0.0087           # 约 0.5° 分辨率
    scan_time: 0.1
    range_min: 0.1
    range_max: 30.0
    use_inf: true
    inf_epsilon: 1.0
```

### 4.2 修改 Nav2 参数

编辑 `src/nav2_bringup_config/config/nav2_params.yaml`：

#### 4.2.1 AMCL 配置（修正坐标系和话题）

找到 `amcl:` 段落，修改为：

```yaml
amcl:
  ros__parameters:
    use_sim_time: False

    # === 坐标系修正 ===
    global_frame_id: "map"            # 全局坐标系（原来是 camera_init，改回 map）
    odom_frame_id: "camera_init"      # FAST_LIO 发布的里程计坐标系
    base_frame_id: "body"             # 机器人本体

    # === 话题修正 ===
    scan_topic: "scan"                # pointcloud_to_laserscan 输出的 /scan
    map_topic: "map"                  # map_server 发布的 /map

    # === 初始位姿 ===
    set_initial_pose: true
    always_reset_initial_pose: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

    # === 其他参数保持不变 ===
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    laser_min_range: -1.0
    laser_max_range: 30.0
    max_beams: 60
    min_particles: 500
    max_particles: 2000
    update_min_d: 0.25
    update_min_a: 0.2
```

#### 4.2.2 代价地图坐标系修正

把 `global_frame` 从 `camera_init` 改为 `map`：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: "map"             # 原来是 camera_init，改为 map
      robot_base_frame: "body"
      # ... 其他保持不变

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: "map"             # 原来是 camera_init，改为 map
      robot_base_frame: "body"
      # ... 其他保持不变
```

#### 4.2.3 里程计话题修正

找到 `bt_navigator` 中的 `odom_topic`，如果配置了的话，改为 FAST_LIO 的里程计：

```yaml
bt_navigator:
  ros__parameters:
    odom_topic: "/Odometry"           # FAST_LIO 发布的里程计话题
    global_frame: "map"
    robot_base_frame: "body"
```

#### 4.2.4 local_costmap obstacle_layer 点云话题确认

如果 obstacle_layer 之前订阅的是 `/cloud_registered_body`，保持即可。FAST_LIO 重启后这个点云依然会正常发布。

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      obstacle_layer:
        pointcloud:
          topic: /cloud_registered_body    # 保持不变
          data_type: PointCloud2
```

### 4.3 修改 Nav2 Bringup Launch

编辑 `src/nav2_bringup_config/launch/nav2_bringup.launch.py`：

#### 修改点 1：支持加载静态地图

把 `map=''` 改为支持外部传入地图路径：

```python
def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_config_dir = get_package_share_directory('nav2_bringup_config')
    
    params_file = os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml')
    
    # === 新增：地图路径参数 ===
    map_yaml_file = LaunchConfiguration('map', default='')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='False')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': use_composition,
            'map': map_yaml_file,        # 原来是 ''，改为变量
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed bringup if True'),
        # === 新增：声明 map 参数 ===
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load'),
        nav2_launch,
    ])
```

---

## 五、新建 Launch 文件

### 5.1 建图专用 Launch

新建 `src/nav2_bringup_config/launch/mapping_with_save.launch.py`：

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 复用现有的 realtime_mapping（FAST_LIO + OctoMap）
    realtime_mapping_launch = os.path.join(
        get_package_share_directory('fast_lio'),
        'launch',
        'realtime_mapping.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realtime_mapping_launch)
        ),
    ])
```

### 5.2 导航专用 Launch（任意位置开机）

新建 `src/nav2_bringup_config/launch/navigation_with_amcl.launch.py`：

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_config_dir = get_package_share_directory('nav2_bringup_config')
    fast_lio_dir = get_package_share_directory('fast_lio')

    # === 参数声明 ===
    map_yaml_file = LaunchConfiguration('map', default='')

    params_file = os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml')
    pc2laser_config = os.path.join(nav2_config_dir, 'config', 'pointcloud_to_laserscan.yaml')

    # 1. Livox 驱动
    livox_launch_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'launch_ROS2',
        'msg_MID360_launch.py'
    )

    # 2. FAST_LIO（纯里程计，不启动 OctoMap）
    fast_lio_launch_path = os.path.join(
        fast_lio_dir,
        'launch',
        'mid360.launch.py'
    )

    # 3. pointcloud_to_laserscan（3D 点云 → 2D scan）
    pc2laser_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[pc2laser_config],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),   # FAST_LIO 输出的 body 坐标系点云
            ('scan', '/scan'),                         # 输出 2D 激光
        ]
    )

    # 4. map_server（加载静态地图）
    # 如果 map 参数为空，则不加载静态地图（回到实时模式）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_file, 'use_sim_time': False}],
        condition=...  # 条件启动，若 map 非空则启动
    )

    # 5. AMCL（全局定位）
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
        }.items()
    )

    # 6. Nav2 导航
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
            'map': map_yaml_file,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load'),

        # 启动传感器 + 里程计
        IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(fast_lio_launch_path)),

        # 3D → 2D
        pc2laser_node,

        # 地图 + 定位 + 导航
        # 注意：map_server 和 amcl 需要在 map 参数非空时启动
        # 如果 map 为空，则只跑 FAST_LIO + Nav2（实时模式）
        # ...
    ])
```

> **说明**：上面的 `map_server` 和 `amcl` 的条件启动需要更精细的处理。如果 `map` 参数为空，走实时模式；如果非空，走 AMCL 静态地图模式。完整的条件判断可用 `IfCondition`。

---

## 六、执行流程

### 阶段一：建图 + 保存地图 + 标记固定点

**终端 1：启动建图**
```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 launch nav2_bringup_config mapping_with_save.launch.py
```

带着雷达走一圈，让 OctoMap 累积出完整地图。

**终端 2：保存 2D 地图**
```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -t /projected_map -f ~/my_map
```

得到：
- `~/my_map.pgm`（栅格图像）
- `~/my_map.yaml`（地图元数据）

**终端 3：标记固定点**

在 RViz2 中：
1. 顶部工具栏点击 **"Publish Point"**
2. 点击自动售货机位置
3. 记录坐标：
   ```bash
   ros2 topic echo /clicked_point
   ```
   输出示例：
   ```
   header:
     frame_id: camera_init
   point:
     x: 5.23
     y: 3.14
     z: 0.0
   ```

4. 把坐标记录到文件 `~/waypoints.yaml`：
   ```yaml
   vending_machine:
     x: 5.23
     y: 3.14
     yaw: 0.0
   ```

> **注意**：这里记录的坐标是 `camera_init` 坐标系下的。由于地图保存时 `camera_init` 和 `map` 原点重合，后续 AMCL 会把 `map` 对齐到保存时的坐标系，所以这个坐标在 `map` 坐标系下同样有效。

### 阶段二：任意位置开机 + 导航

**终端 1：启动导航（加载静态地图）**
```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 launch nav2_bringup_config navigation_with_amcl.launch.py map:=/home/zws/my_map.yaml
```

启动后：
1. FAST_LIO 开始运行，`camera_init` 在新的位置重置
2. `pointcloud_to_laserscan` 把 `/cloud_registered_body` 转成 `/scan`
3. `map_server` 加载 `~/my_map.yaml`，发布 `/map`
4. AMCL 订阅 `/scan` 和 `/map`，开始粒子滤波定位
5. Nav2 启动，等待导航目标

**初始定位**：

AMCL 启动时可能不知道自己在哪。有两种方式给初始位姿：

方式 A：RViz2 中点击 **"2D Pose Estimate"**，在地图上拖动指定机器人当前的大致位置和朝向。

方式 B：命令行发布初始位姿：
```bash
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

> 如果不知道当前大致位置，可以把 `x, y` 设为地图中心。AMCL 会在全局撒粒子，机器人走几步后就会收敛。

**终端 2：发送导航目标**

导航到固定点（自动售货机）：
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.23, y: 3.14, z: 0.0},
    orientation: {z: 0.0, w: 1.0}
  }
}" --once
```

或者写一个 Python 脚本用 `nav2_simple_commander` 读取 `waypoints.yaml` 自动发送。

---

## 七、编译

修改完配置和 launch 文件后：

```bash
cd /home/zws/ws_livox
colcon build --packages-select nav2_bringup_config
source install/setup.bash
```

---

## 八、验证清单

| 检查项 | 命令 | 预期结果 |
|--------|------|---------|
| `/scan` 是否正常发布 | `ros2 topic hz /scan` | 10 Hz 左右 |
| `/map` 是否正常发布 | `ros2 topic hz /map` | 有频率 |
| AMCL 是否发布 TF | `ros2 run tf2_tools view_frames` | 看到 `map → camera_init → body` 链 |
| 代价地图是否有障碍物 | RViz2 中添加 `/local_costmap/costmap` | Color Scheme 选 `costmap`，能看到红色障碍物 |
| 固定点坐标是否正确 | 发送 goal_pose 后机器人是否朝正确方向走 | 机器人应向目标点移动 |

---

## 九、常见问题

### 9.1 AMCL 报 "No map received"
- 检查 `map_server` 是否启动
- 检查 `map` 参数路径是否正确
- 检查 `.yaml` 文件中 `image` 字段指向的 `.pgm` 文件是否存在

### 9.2 `/scan` 没有数据
- 检查 `pointcloud_to_laserscan` 是否启动
- 检查 `/cloud_registered_body` 是否有数据：`ros2 topic hz /cloud_registered_body`
- 检查 `pointcloud_to_laserscan.yaml` 中的 `min_height`/`max_height` 是否过滤掉了所有点

### 9.3 AMCL 定位发散（粒子云越来越大）
- 在 RViz2 中给初始位姿（"2D Pose Estimate"）
- 检查 `/scan` 和地图是否匹配（环境是否变化太大）
- 降低 `laser_min_range`，增加 `laser_max_range`

### 9.4 机器人走到一半说 "Failed to reach goal"
- 检查 local_costmap 是否看到实时障碍物
- 检查 `global_frame` 是否都改成了 `map`
- 检查固定点坐标是否基于 `map` 坐标系

### 9.5 TF 树断开
- 检查 AMCL 是否正常运行
- 检查 `global_frame_id: map`, `odom_frame_id: camera_init`, `base_frame_id: body` 是否正确
- 检查 FAST_LIO 是否正常发布 `camera_init → body`

---

## 十、文件修改汇总

| 文件 | 操作 | 说明 |
|------|------|------|
| `src/nav2_bringup_config/config/pointcloud_to_laserscan.yaml` | 新建 | 3D→2D 转换参数 |
| `src/nav2_bringup_config/config/nav2_params.yaml` | 修改 | AMCL 坐标系、代价地图坐标系、里程计话题 |
| `src/nav2_bringup_config/launch/nav2_bringup.launch.py` | 修改 | 支持 `map` 参数传入 |
| `src/nav2_bringup_config/launch/mapping_with_save.launch.py` | 新建 | 建图阶段启动 |
| `src/nav2_bringup_config/launch/navigation_with_amcl.launch.py` | 新建 | 导航阶段启动（含 pointcloud_to_laserscan + AMCL） |

---

## 十一、后续优化方向

- [ ] 用 `nav2_simple_commander` 写 Python 脚本，读取 `waypoints.yaml` 实现一键导航到多个固定点
- [ ] 给 `navigation_with_amcl.launch.py` 加条件判断：`map` 参数为空时走实时模式，非空时走 AMCL 模式
- [ ] 如果 2D AMCL 精度不够，可考虑 3D 点云定位（如 `fast_lio_localization` ROS2 版本）
- [ ] 保存地图时同时记录固定点坐标到 `.yaml`，启动时自动加载
