# Nav2 静态地图模式问题记录

## 已解决的问题 ✅

1. ✅ **AMCL 静态地图模式实时避障原理已明确**
   - 静态地图用于 AMCL 定位和全局规划，实时避障由 local_costmap 的 obstacle_layer 处理，两者互不冲突。

2. ✅ **静态地图模式启动命令已明确**
   - 加载地图：`ros2 launch nav2_bringup_config navigation_with_amcl.launch.py map:=/home/zws/my_map.yaml`
   - 纯实时模式（不加载地图）：`ros2 launch nav2_bringup_config navigation_with_amcl.launch.py`

3. ✅ **地图保存命令已明确**
   - `ros2 run nav2_map_server map_saver_cli -t /projected_map -f ~/my_map`
   - 注意：保存前需启动 octomap_server 发布 `/projected_map`。

4. ✅ **map_server 重复启动导致 `/map` 无发布者的问题已修复**
   - 原因：`navigation_with_amcl.launch.py` 自建的 map_server_node 与 `localization_launch.py`、`navigation_launch.py` 启动的 map_server 冲突。
   - 修复：删除自建的 `map_server_node`，让 `localization_launch.py` 统一管理 amcl + map_server；`navigation_launch.py` 不再传 `map` 参数。

5. ✅ **`nav2_params.yaml` 缺少 map_server 配置的问题已修复**
   - 原因：`localization_launch.py` 依赖 `RewrittenYaml` 重写 `yaml_filename`，但原 YAML 中没有该键。
   - 修复：在 `nav2_params.yaml` 中添加 `map_server` 节点配置。

6. ✅ **代价地图订阅错误话题的问题已修复**
   - 原因：`global_costmap` 和 `local_costmap` 的 `static_layer` 订阅 `/projected_map`，但静态地图模式下应订阅 `/map`。
   - 修复：将两处 `map_topic` 从 `/projected_map` 改为 `/map`。

7. ✅ **RViz 中 Map 插件显示异常（感叹号）的问题已解决**
   - 原因：QoS Durability 不匹配，map_server 使用 `Transient Local`。
   - 解决：RViz 中 Map 插件的 Durability Policy 设为 `Transient Local`。

---

## 待解决的问题 ❌

### 1. ❌ AMCL 定位收敛困难 —— `/scan` 与 `/map` 不匹配
**现象**：
- 启动静态地图模式后，AMCL 位姿收敛到 `(-8.66, 2.80)`，但该位置不一定是机器人真实位置。
- RViz 中 `/scan`（LaserScan）激光线与 `/map` 静态地图的墙壁/障碍物明显不重合。
- 机器人静止 5~10 分钟后仍无法自动对齐。

**根因分析**：
- FAST_LIO 每次重启后 `camera_init` 坐标系原点重置为 `(0,0)`。
- 保存的静态地图 `origin: [-23.5, -15.3]` 是地图边界框左下角，与 `camera_init` 原点无直接关联。
- AMCL 的 `initial_pose: (0,0,0)` 仅为初始猜测，若实际环境特征与地图匹配度低（或存在相似走廊），AMCL 容易陷入局部最优或收敛到错误位置。

**影响**：
- **导航路线会偏移**。AMCL 负责计算 `map -> camera_init` 的变换，若该变换错误，全局坐标系下所有位置（包括目标点、路径、代价地图）都会产生偏移。
- 局部避障（local_costmap）不受直接影响，因为避障依赖实时点云 `/cloud_registered_body`，与 AMCL 定位无关。

**当前状态**：
- 2D Pose Estimate 手动对齐操作困难，用户反馈难以准确点击位置和方向。
- 尚未找到让 AMCL 在启动位置固定时自动收敛到正确位置的配置方案。

**可能的解决方向**：
- 若机器人每次在固定位置启动，记录 AMCL 收敛后的 `/amcl_pose`，写入 `nav2_params.yaml` 的 `initial_pose` 作为默认值。
- 调整 AMCL 参数（如增加 `max_particles`、`laser_model_type` 改为 `beam` 等）提高收敛鲁棒性。
- 检查 `/scan` 数据质量（pointcloud_to_laserscan 的高度范围、角度分辨率）是否足够支持 AMCL 匹配。

---

### 2. ❌ FAST_LIO 全局点云地图 `/Laser_map` 混乱
**现象**：
- RViz 中 `/Laser_map` 显示大量散乱的点云，与 `/map` 静态地图完全不重合。
- 视觉上地图"非常烂和乱"。

**根因分析**：
- 之前多次尝试 2D Pose Estimate 手动对齐时，AMCL 位姿错误，导致 FAST_LIO 将新扫描点云注册到了错误的全局位置。
- FAST_LIO 的 `/Laser_map` 会累积历史点云，不会自动清除错误数据。

**影响**：
- **对导航零影响**。Nav2 导航栈不使用 `/Laser_map`。
- 仅影响 RViz 可视化，可能误导用户对系统状态的判断。

**当前状态**：
- 用户已了解 `/Laser_map` 混乱不影响导航，但视觉上造成困扰。
- 尚无快速清除 `/Laser_map` 的方法（需重启 FAST_LIO 节点，但会同时丢失里程计）。

**可能的解决方向**：
- 在 RViz 配置中移除 `/Laser_map` 显示，避免视觉干扰。
- 如需清理，可在启动导航前单独重启 FAST_LIO（但需确保 AMCL 和代价地图随后正确初始化）。

---

### 3. ❌ 坐标系原点不一致导致的理解困惑
**现象**：
- 用户 intuitively 认为"3D点云图应该和2D地图重合"，但实际两者坐标系原点不同，视觉上自然不重合。
- 用户担心"原点没重合会导致导航路线偏移"。

**根因分析**：
- FAST_LIO 的 `camera_init` 是**相对里程计坐标系**，每次重启原点重置。
- `/map` 是**全局静态坐标系**，origin 由保存地图时的环境边界决定。
- 两者的对齐完全依赖 AMCL 实时计算 `map -> camera_init` 变换，而非固定原点。

**澄清**：
- "点云图和2D地图是否视觉上重合" ≠ "导航是否准确"。
- 导航精度只取决于 AMCL 是否收敛正确（即 `/scan` 激光线是否压在 `/map` 墙壁上）。
- 只要 AMCL 收敛正确，`/cloud_registered_body` 实时点云会通过 TF 正确显示在 `map` 坐标系中，与静态地图对齐。

**当前状态**：
- 已通过文档和对话向用户解释，但实际操作中用户仍难以直观判断 AMCL 是否正确收敛。

---

## 快速诊断清单

启动后检查 AMCL 是否正确收敛：

1. `ros2 topic echo /amcl_pose --once` —— 记录位姿，观察是否稳定。
2. RViz 中对比 `/scan`（LaserScan）和 `/map`（Map）—— 激光线是否压在墙壁/障碍物上。
3. `ros2 run tf2_ros tf2_echo map body` —— 检查 TF 是否稳定，无剧烈跳变。
4. 若以上任一项异常，使用 RViz "2D Pose Estimate" 工具手动重新对齐。
