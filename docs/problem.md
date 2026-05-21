# FAST-LIO 里程计精度、局限性与输出方式

> 本文档说明 FAST-LIO 输出的位姿/运动信息是否可信，如何获取这些数据，以及在实际导航任务（如往返饮水机）中需要注意的问题。

---

## 1. FAST-LIO 的输出准确吗？

**短距离很准，长距离会漂，退化场景会失效。**

FAST-LIO 在典型的室内/结构化环境里，精度表现如下：

| 场景 | 精度表现 | 原因 |
|------|---------|------|
| **短距离（< 100m，几分钟内）** | **很高**，位置误差通常 < 1%（走 50m 误差可能不到 0.5m） | IMU + 点云紧耦合，局部地图丰富 |
| **长距离（> 500m 或长时间运行）** | **会累积漂移**，可能漂几米甚至十几米 | **没有回环检测**，误差不断积分 |
| **特征退化场景**（长走廊、空旷大厅、四面白墙） | **可能突然跳变或跟踪丢失** | 点云配准找不到足够约束，IEKF 更新失效 |
| **剧烈运动/快速旋转** | **优于纯点云方法**，但仍有微小漂移 | IMU 补偿了运动畸变，但高动态下 IMU 噪声也会被放大 |

> **关键点**：FAST-LIO 输出的是 **`camera_init` 坐标系下的绝对位姿**，不是"从上一帧走了多少"的相对量。这个绝对位姿在启动瞬间被固定为原点 `(0,0,0)`，之后所有坐标都是相对于这个原点的。

### "运动了多少" vs "运动到哪里"

| 概念 | 含义 | 怎么得到 |
|------|------|---------|
| **运动到哪里** | 当前在 `camera_init` 世界坐标系中的绝对位置 | 直接读 `/Odometry.pose.pose` |
| **运动了多少** | 从起点到现在的相对位移和角度变化 | 需要自己算：当前位姿 － 起始位姿 |
| **瞬时速度** | 当前时刻的线速度和角速度 | 直接读 `/Odometry.twist.twist` |

---

## 2. 怎么输出运动信息？

### 2.1 命令行直接查看（最快捷）

```bash
# 查看完整位姿（position + orientation）
ros2 topic echo /Odometry

# 只看 x, y 坐标
ros2 topic echo /Odometry/pose/pose/position/x
ros2 topic echo /Odometry/pose/pose/position/y

# 查看当前速度（线速度 m/s，角速度 rad/s）
ros2 topic echo /Odometry/twist/twist/linear/x
ros2 topic echo /Odometry/twist/twist/angular/z

# 查看 TF 变换
cd /tmp && ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo camera_init body
```

### 2.2 写节点订阅并计算（推荐用于自定义脚本）

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')
        self.subscription = self.create_subscription(
            Odometry, '/Odometry', self.odom_callback, 10)
        self.start_pose = None

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 四元数转 yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # 记录起点
        if self.start_pose is None:
            self.start_pose = (x, y, yaw)
            self.get_logger().info(f"起点已记录: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        # 计算相对位移
        dx = x - self.start_pose[0]
        dy = y - self.start_pose[1]
        dist = math.sqrt(dx**2 + dy**2)

        # 瞬时速度
        vx = msg.twist.twist.linear.x
        vz = msg.twist.twist.angular.z

        self.get_logger().info(
            f"当前: x={x:.3f}, y={y:.3f}, 已走={dist:.3f}m, "
            f"速度={vx:.3f}m/s, 角速={vz:.3f}rad/s"
        )

rclpy.init()
node = OdomMonitor()
rclpy.spin(node)
```

### 2.3 保存日志或转发给其他节点

如果需要持久化数据或跨节点通信：

```python
def odom_callback(self, msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # 追加写入日志文件
    with open('/tmp/odom_log.txt', 'a') as f:
        f.write(f"{timestamp:.3f} {x:.4f} {y:.4f}\n")

    # 或者发布到自定义话题供其他节点订阅
    # self.pub.publish(...)
```

---

## 3. 对实际导航任务的影响（以"去饮水机再返回"为例）

| 风险 | 说明 | 建议对策 |
|------|------|---------|
| **返回时定位漂移** | 如果去饮水机路程较远（> 100m），FAST-LIO 的累积漂移可能导致回来时偏离原位 | 设置一个**停靠容忍区域**（如半径 0.3m），不必像素级对准；或定期重启 FAST-LIO 重置原点 |
| **中途被抱起来/碰撞** | FAST-LIO 是里程计，不是全局定位。机器人被突然搬动后，地图与真实环境错位，后续定位会错乱 | 尽量避免；一旦发生只能重启 FAST-LIO |
| **饮水机旁环境单调** | 如果饮水机位于一面白墙或空旷区域附近，点云配准可能退化，导致定位跳变 | 确保饮水机周围有几何结构（墙角、柜子、门框），否则可能丢失跟踪 |
| **长时间运行漂移** | 无回环检测，运行越久误差越大 | 场景不大时问题不大；大场景建议先建图保存，再切换为定位模式 |

---

## 4. 一句话总结

- **"运动到哪里"**：直接读 `/Odometry.pose.pose`，是 `camera_init` 下的绝对坐标，短距离很准。
- **"运动了多少"**：用当前位姿减起始位姿自己算，或者从 `/Odometry.twist` 读取瞬时速度再积分。
- **输出方式**：`ros2 topic echo` 最省事，写个 Python 节点订阅最灵活。
- **核心局限**：没有回环检测，长距离会漂移；退化场景可能丢跟踪。

---

*文档生成时间：2026-04-23*  
*对应配置版本：`src/FAST_LIO_ROS2/launch/realtime_mapping.launch.py`*
