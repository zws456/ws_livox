# 底盘 UART 串口控制指南

> 本文档说明如何通过 UART 串口将 Nav2 导航输出的速度指令发送给下位机（STM32），以及完整的建图→导航→控制流程。

---

## 一、系统架构

```
┌─────────────────┐      /cmd_vel      ┌──────────────────┐     UART(TX/RX)     ┌─────────────┐
│   Nav2 导航      │ ─────────────────→ │ cmd_vel_to_uart  │ ─────────────────→ │   STM32     │
│  (路径规划)       │   geometry_msgs    │   (ROS2 节点)     │   115200 8N1      │  (电控负责)  │
│                 │      /Twist        │                  │                   │             │
└─────────────────┘                    └──────────────────┘                   └─────────────┘
                                                                                     │
                                                                                     ↓
                                                                               ┌─────────────┐
                                                                               │  全向轮电机  │
                                                                               └─────────────┘
```

---

## 二、UART 串口协议（给电控同学看）

### 2.1 帧格式

定长 **16 字节**，小端（Little-Endian）：

| 字节偏移 | 长度 | 类型 | 内容 | 说明 |
|---------|------|------|------|------|
| 0 | 1 | uint8 | `0xAA` | 帧头 |
| 1 | 1 | uint8 | `0x55` | 帧头 |
| 2~5 | 4 | float32 LE | f1 | 数据 1 |
| 6~9 | 4 | float32 LE | f2 | 数据 2 |
| 10~13 | 4 | float32 LE | f3 | 数据 3 |
| 14 | 1 | uint8 | checksum | 校验：前 14 字节异或 |
| 15 | 1 | uint8 | `0x0D` | 帧尾 |

### 2.2 校验算法（C 语言）

```c
uint8_t checksum = 0;
for (int i = 0; i < 14; i++) {
    checksum ^= frame[i];
}
```

### 2.3 数据含义

#### 模式 A（默认，推荐）

| 字段 | 物理量 | 单位 | 正负定义 |
|------|--------|------|----------|
| f1 | `vx` | m/s | **+ 前进**，- 后退 |
| f2 | `vy` | m/s | **+ 左移**，- 右移（全向轮） |
| f3 | `omega` | rad/s | **+ 逆时针转**，- 顺时针转 |

> 电控同学拿到后直接代入全向轮/麦轮运动学逆解公式。

#### 模式 B（极坐标形式）

如需发送"方向+速度+转动"语义：

| 字段 | 物理量 | 单位 | 说明 |
|------|--------|------|------|
| f1 | `heading` | rad | 前进方向，`0`=正前，`π/2`=正左 |
| f2 | `speed` | m/s | 合速度，始终 ≥ 0 |
| f3 | `omega` | rad/s | 自转角速度 |

> 如需模式 B，在 ROS2 节点代码中取消对应注释即可。

### 2.4 波特率

- **默认**：`115200`（和 walking-table STM32 配置一致）
- 如需修改，ROS2 端和 STM32 端必须改成**同一个值**

---

## 三、ROS2 节点使用

### 3.1 确认串口设备名

插上 USB 转串口模块后，先确认系统识别的设备名：

```bash
ls /dev/ttyUSB*
```

通常会显示：
```
/dev/ttyUSB0
/dev/ttyUSB1
```

如果只有一个模块，可能只有 `/dev/ttyUSB0`。如分不清哪个是哪个，拔插其中一个看变化。

### 3.2 串口权限

首次使用需给串口设备权限：

```bash
sudo chmod 666 /dev/ttyUSB0
# 如果有多个：
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
```

或永久解决（需重新登录生效）：

```bash
sudo usermod -a -G dialout $USER
```

### 3.3 启动命令

```bash
cd /home/zws/ws_livox
source install/setup.bash

# 默认参数（/dev/ttyUSB0，115200，20Hz）
ros2 run nav2_bringup_config cmd_vel_to_uart

# 指定串口设备（如识别为 ttyUSB1）
ros2 run nav2_bringup_config cmd_vel_to_uart --ros-args -p device:=/dev/ttyUSB1

# 指定波特率（如电控改成了 9600）
ros2 run nav2_bringup_config cmd_vel_to_uart --ros-args -p baudrate:=9600

# 指定发送频率（默认 20Hz，和 Nav2 controller_frequency 一致）
ros2 run nav2_bringup_config cmd_vel_to_uart --ros-args -p publish_rate:=50.0
```

### 3.4 查看发送的数据

```bash
# 查看 Nav2 原始输出
ros2 topic echo /cmd_vel

# 只看前进速度
ros2 topic echo /cmd_vel/linear/x

# 只看旋转速度
ros2 topic echo /cmd_vel/angular/z
```

---

## 四、协议验证：双 USB 串口回环测试

在把 UART 线接到真正的 STM32 之前，建议先用**两个 USB 转串口模块**做回环测试，验证 ROS2 节点发的数据格式是否正确。

### 4.1 硬件接线

准备两个 USB 转 TTL 串口模块，**TX/RX 交叉连接，GND 共地**：

```
USB转串口 A (接电脑，跑 ROS2 节点)       USB转串口 B (接电脑，用来抓包)
┌─────────────────┐                      ┌─────────────────┐
│  TX  ───────────┼────────────────────→│  RX             │
│  RX  ←──────────┼─────────────────────│  TX             │
│  GND ───────────┼─────────────────────│  GND            │
└─────────────────┘                      └─────────────────┘
```

### 4.2 准备工作

按 **3.1 确认设备名** 和 **3.2 给权限** 的步骤操作好。假设两个模块分别为：
- `/dev/ttyUSB0` —— 接 ROS2 发送节点
- `/dev/ttyUSB1` —— 接抓包脚本

### 4.3 终端 1：启动抓包（监听 ttyUSB1）

**推荐用 Python 脚本抓包**（可直接解析协议）：

```python
#!/usr/bin/env python3
import serial
import struct

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
print("Listening on /dev/ttyUSB1...")

while True:
    raw = ser.read(16)
    if len(raw) == 16:
        if raw[0] == 0xAA and raw[1] == 0x55 and raw[15] == 0x0D:
            checksum = 0
            for i in range(14):
                checksum ^= raw[i]
            if checksum == raw[14]:
                f1, f2, f3 = struct.unpack('<fff', raw[2:14])
                print(f"✓ Valid: vx={f1:.3f}, vy={f2:.3f}, omega={f3:.3f}")
            else:
                print(f"✗ Checksum error: {raw.hex()}")
        else:
            print(f"? Bad frame: {raw.hex()}")
```

保存为 `uart_sniffer.py`，运行：

```bash
python3 uart_sniffer.py
```

> 也可用 `cat /dev/ttyUSB1 | xxd -g 1` 直接看十六进制原始流。

### 4.4 终端 2：启动 ROS2 串口发送节点（发给 ttyUSB0）

```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 run nav2_bringup_config cmd_vel_to_uart --ros-args -p device:=/dev/ttyUSB0
```

### 4.5 终端 3：手动发布测试速度（模拟 Nav2）

```bash
# 发一个固定速度：前进 0.5m/s，左移 0.2m/s，逆时针转 0.3rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.5, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}" \
  --rate 20
```

### 4.6 预期结果

抓包终端应持续输出：

```
✓ Valid: vx=0.500, vy=0.200, omega=0.300
✓ Valid: vx=0.500, vy=0.200, omega=0.300
...
```

用 `xxd` 看十六进制应类似：

```
aa 55 00 00 00 3f cd cc 4c 3e 9a 99 99 3e ?? 0d
```

含义：
- `aa 55`：帧头
- `00 00 00 3f`：`0.5` 的小端 float32
- `cd cc 4c 3e`：`0.2` 的小端 float32
- `9a 99 99 3e`：`0.3` 的小端 float32
- `??`：校验和
- `0d`：帧尾

### 4.7 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| 抓包端完全无输出 | TX/RX 没交叉、GND 没连、波特率不一致 | 检查接线；确认两边都是 115200 |
| 输出乱码 | 波特率不匹配 | 统一波特率 |
| 有数据但帧头不是 `aa 55` | 线路干扰或数据错位 | 换短杜邦线；检查 ROS2 端 `publish_rate` 是否过高 |
| 校验失败 | checksum 算法不一致 | 确认是"前 14 字节异或" |

---

---

## 五、完整操作流程

### Step 1：建图（扫描新环境）

```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 launch fast_lio realtime_mapping.launch.py
```

- 会自动弹出 RViz2
- 带着雷达在新环境走一圈，等 2D 栅格地图完整

### Step 2：保存 2D 地图

另开终端：

```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -t /projected_map -f ~/new_map
```

生成：
- `~/new_map.pgm` —— 地图图片
- `~/new_map.yaml` —— 地图配置

**注意**：yaml 里的 `image` 路径是相对路径，建议改为绝对路径：

```bash
sed -i 's|image: new_map.pgm|image: /home/zws/new_map.pgm|' ~/new_map.yaml
```

### Step 3：启动导航 + 串口发送

**终端 1：导航（加载地图）**

```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 launch nav2_bringup_config navigation_with_amcl.launch.py map:=/home/zws/new_map.yaml
```

**终端 2：串口发送节点**

```bash
cd /home/zws/ws_livox
source install/setup.bash
ros2 run nav2_bringup_config cmd_vel_to_uart
```

**终端 3：RViz2（看地图、标点）**

```bash
ros2 run rviz2 rviz2
```

### Step 4：初始定位（⚠️ 关键）

导航启动后，AMCL 不知道机器人在地图上的实际位置。在 RViz2 中：

1. 点击顶部工具栏 **"2D Pose Estimate"**
2. 在地图上**拖动**，指定：
   - **位置**：机器人当前实际在哪里
   - **朝向**：机器人现在面朝哪个方向

AMCL 收到初始猜测后，会把雷达扫描和地图匹配，确定全局位置。

### Step 5：标目标点

1. 点击 **"Nav2 Goal"**（或 **"2D Goal Pose"**）
2. 在地图上拖动，指定想去的位置和朝向

Nav2 会自动规划路径，发布 `/cmd_vel`，串口节点自动发给 STM32，机器人开始移动。

---

## 六、运动学参考（基于 walking-table 仓库）

### 5.1 全向轮逆解公式

walking-table 采用的是 **60° 安装的四全向轮**底盘：

```c
// 底盘参数
const float sin60 = 0.866f;
const float cos60 = 0.5f;
const float R = 1.02f;           // 旋转半径（需根据实际底盘调整）
const float wheel_radius = 0.05f; // 轮子半径（需根据实际调整）

// 逆解（输出为各电机角速度 rad/s）
motor[0] = ( vx * sin60 - vy * cos60 - omega * R) / wheel_radius;  // 前左
motor[1] = (-vx * sin60 - vy * cos60 - omega * R) / wheel_radius;  // 前右
motor[2] = ( vx * sin60 + vy * cos60 - omega * R) / wheel_radius;  // 后左
motor[3] = (-vx * sin60 + vy * cos60 - omega * R) / wheel_radius;  // 后右
```

### 5.2 标准四麦轮（45° 安装）逆解公式

如果你的底盘是**标准四麦轮**（Mecanum），公式不同：

```c
// X 型布局（前左、前右、后左、后右）
// L = 前后轮距/2, W = 左右轮距/2
float k = L + W;

motor[0] =  vx - vy - omega * k;  // 前左
motor[1] =  vx + vy - omega * k;  // 前右
motor[2] = -vx - vy + omega * k;  // 后左（注意电机安装朝向）
motor[3] = -vx + vy + omega * k;  // 后右
```

> ⚠️ **务必确认实际底盘类型和轮子安装角度**，套错公式轮子会乱转。

---

## 七、常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 串口打不开 `/dev/ttyUSB0` | 权限不足或设备名不对 | `sudo chmod 666 /dev/ttyUSB0`，或确认设备名 `ls /dev/ttyUSB*` |
| STM32 收不到数据 | 波特率不匹配 | 两边统一成同一个波特率 |
| 数据校验失败 | 帧格式对不上 | 确认小端/大端、帧头帧尾、校验算法一致 |
| 机器人乱转/轮子不同步 | 运动学公式和底盘不匹配 | 确认是全向轮 60° 还是麦轮 45° |
| Nav2 不发 `/cmd_vel` | 没有给定目标点，或定位丢失 | 在 RViz 里重新给初始位姿和目标点 |

---

## 八、后续优化方向

当前链路是**单向开环**的：

```
Nav2 → /cmd_vel → 串口 → STM32 → 电机
```

Nav2 不知道机器人实际走了多远。如需提升导航精度，建议电控同学加：

1. **编码器测速**：测每个轮子的实际转速
2. **里程计回传**：STM32 把实际速度通过串口发回 ROS2
3. **ROS2 端**：publish 实际里程计，供 Nav2 做闭环校正

---

## 九、文件清单

| 文件 | 作用 |
|------|------|
| `src/nav2_bringup_config/scripts/cmd_vel_to_uart.py` | ROS2 串口发送节点 |
| `src/nav2_bringup_config/CMakeLists.txt` | 配置节点安装 |
| `~/new_map.pgm / .yaml` | 保存的 2D 地图 |
