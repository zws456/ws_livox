#!/usr/bin/env python3
"""
cmd_vel_to_uart.py

订阅 ROS2 /cmd_vel，通过 UART 发送运动控制数据给下位机（STM32）。

协议格式（16 字节定长帧，小端，无校验）：
    [0xAA] [0x55] [float32] [float32] [float32] [0x0D] [0x0A]
    
默认发送内容（模式 A —— 推荐，电控同学可直接做运动学逆解）：
    float[0] = vx     (m/s)   前进速度，正为前进
    float[1] = vy     (m/s)   左右速度，正为左移（全向轮）
    float[2] = omega  (rad/s) 旋转角速度，正为逆时针

如需模式 B（方向 + 速度 + 转动），见代码第 70~73 行。

用法：
    ros2 run nav2_bringup_config cmd_vel_to_uart --ros-args -p device:=/dev/ttyUSB0
    
    改发送频率（默认 20Hz）：
    ros2 run nav2_bringup_config cmd_vel_to_uart --ros-args -p publish_rate:=50.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import math


class CmdVelToUart(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_uart')

        # 参数
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 20.0)

        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        publish_rate = self.get_parameter('publish_rate').value

        # 打开串口
        try:
            self.ser = serial.Serial(device, baudrate, timeout=0.01)
            self.get_logger().info(f'UART opened: {device} @ {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open UART {device}: {e}')
            raise

        # 订阅 Nav2 发布的 /cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        # 固定发送频率定时器
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # 缓存最新速度
        self.latest_f1 = 0.0
        self.latest_f2 = 0.0
        self.latest_f3 = 0.0
        self.has_new_data = False

        self.get_logger().info(
            f'Subscribed to /cmd_vel, UART publish rate: {publish_rate:.1f} Hz'
        )

    def on_cmd_vel(self, msg: Twist):
        """收到 /cmd_vel 时只缓存数据，不直接发串口。"""
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        omega = float(msg.angular.z)

        # 模式 A（默认）：发笛卡尔分量 vx, vy, omega
        self.latest_f1, self.latest_f2, self.latest_f3 = vx, vy, omega

        # 模式 B（极坐标）：如果电控要"方向+速度+转动"，取消下面注释
        # self.latest_f1 = math.atan2(vy, vx)   # heading (rad)
        # self.latest_f2 = math.hypot(vx, vy)   # speed (m/s)
        # self.latest_f3 = omega                # omega (rad/s)

        self.has_new_data = True

    def timer_callback(self):
        """定时器回调：按固定频率发送最新缓存的速度。"""
        if not self.has_new_data:
            return

        frame = self.pack_frame(self.latest_f1, self.latest_f2, self.latest_f3)
        try:
            self.ser.write(frame)
            hex_str = ' '.join(f'{b:02x}' for b in frame)
            self.get_logger().info(
                f'Sent Hex: {hex_str}'
            )
        except serial.SerialException as e:
            self.get_logger().warn(f'UART write failed: {e}')

    def pack_frame(self, f1: float, f2: float, f3: float) -> bytes:
        """
        打包协议帧（无校验位，16 字节定长）：
        [0xAA][0x55][f1:float32][f2:float32][f3:float32][0x0D][0x0A]
        """
        data = bytearray(16)
        data[0] = 0xAA
        data[1] = 0x55
        struct.pack_into('<f', data, 2, f1)
        struct.pack_into('<f', data, 6, f2)
        struct.pack_into('<f', data, 10, f3)
        data[14] = 0x0D
        data[15] = 0x0A

        return bytes(data)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('UART closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToUart()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
