#!/usr/bin/env python3
"""
cmd_vel_to_uart.py

订阅 ROS2 /cmd_vel，通过 UART 发送运动控制数据给下位机（STM32）。
同时读取底盘回传的遥测数据（ASCII 文本协议），发布到 ROS2 Topic。

发送协议（16 字节定长帧，小端）：
    [0xAA] [0x55] [float32] [float32] [float32] [0x0D] [0x0A]
    
接收协议（6 行 ASCII 文本，以 \n 分隔）：
    mode:xxx
    vx:0.00 | vy:0.00 | omega:0.00
    m0:tgt:100 | fdb:95 | pid_out:... | kp:... | ki:... | kd:... | e:... | p:... | i:... | d:...
    m1:tgt:100 | fdb:95 | pid_out:... | kp:... | ki:... | kd:... | e:... | p:... | i:... | d:...
    m2:tgt:100 | fdb:95 | pid_out:... | kp:... | ki:... | kd:... | e:... | p:... | i:... | d:...
    m3:tgt:100 | fdb:95 | pid_out:... | kp:... | ki:... | kd:... | e:... | p:... | i:... | d:...

用法：
    ros2 run nav2_bringup_config cmd_vel_to_uart --ros-args -p device:=/dev/ttyUSB0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
import serial
import struct
import math
import threading
import json


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

        # 发布底盘遥测数据和反馈速度
        self.telemetry_pub = self.create_publisher(String, '/chassis_telemetry', 10)
        self.feedback_pub = self.create_publisher(TwistStamped, '/chassis_feedback', 10)

        # 固定发送频率定时器
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # 缓存最新速度
        self.latest_f1 = 0.0
        self.latest_f2 = 0.0
        self.latest_f3 = 0.0
        self.has_new_data = False

        # 接收缓冲区
        self.recv_lines = []
        self.recv_lock = threading.Lock()

        # 启动读取线程
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(
            f'Subscribed to /cmd_vel, UART publish rate: {publish_rate:.1f} Hz, '
            f'Telemetry topics: /chassis_telemetry, /chassis_feedback'
        )

    def on_cmd_vel(self, msg: Twist):
        """收到 /cmd_vel 时只缓存数据，不直接发串口。"""
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        omega = float(msg.angular.z)

        # 坐标系转换：body(雷达) 相对底盘顺时针 90°
        # body x = 底盘右方, body y = 底盘前方
        # 底盘期望：f1=前进(vy), f2=左移(-vx)
        self.latest_f1 = -vy      # 底盘前进 = -body y（修正方向）
        self.latest_f2 = vx       # 底盘左移 = body x
        self.latest_f3 = omega

        self.has_new_data = True

    def timer_callback(self):
        """定时器回调：按固定频率发送最新缓存的速度。"""
        if not self.has_new_data:
            return

        frame = self.pack_frame(self.latest_f1, self.latest_f2, self.latest_f3)
        try:
            self.ser.write(frame)
            hex_str = ' '.join(f'{b:02x}' for b in frame)
            self.get_logger().info(f'Sent Hex: {hex_str}')
        except serial.SerialException as e:
            self.get_logger().warn(f'UART write failed: {e}')

    def read_loop(self):
        """后台线程：持续读取串口回传的 ASCII 遥测数据。"""
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                with self.recv_lock:
                    if line.startswith('mode:'):
                        self.recv_lines = [line]
                    elif self.recv_lines:
                        self.recv_lines.append(line)
                        if len(self.recv_lines) >= 6:
                            self.parse_and_publish(self.recv_lines[:6])
                            self.recv_lines = []
            except Exception as e:
                self.get_logger().warn(f'UART read error: {e}')
                break

    def parse_and_publish(self, lines):
        """解析 6 行遥测数据并发布到 ROS2。"""
        try:
            # 第 1 行: mode:xxx
            mode = lines[0].split(':', 1)[1].strip()

            # 第 2 行: vx:0.00 | vy:0.00 | omega:0.00
            parts = [p.strip() for p in lines[1].split('|')]
            vx = float(parts[0].split(':', 1)[1])
            vy = float(parts[1].split(':', 1)[1])
            omega = float(parts[2].split(':', 1)[1])

            # 第 3-6 行: m0-m3 电机数据
            motors = []
            for i in range(4):
                motor_data = {}
                for item in lines[i + 2].split(' | '):
                    tokens = item.split(':')
                    key = tokens[-2].strip()
                    value = tokens[-1].strip()
                    motor_data[key] = value
                motors.append(motor_data)

            # 发布 JSON 遥测数据
            telemetry = {
                'mode': mode,
                'vx': vx,
                'vy': vy,
                'omega': omega,
                'motors': motors
            }
            msg = String()
            msg.data = json.dumps(telemetry, ensure_ascii=False)
            self.telemetry_pub.publish(msg)

            # 发布 TwistStamped 反馈速度
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'body'
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            twist.twist.angular.z = omega
            self.feedback_pub.publish(twist)

            self.get_logger().info(
                f'[RECV] mode={mode} vx={vx:.3f} vy={vy:.3f} omega={omega:.3f}'
            )
        except Exception as e:
            self.get_logger().warn(f'Telemetry parse error: {e}')

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
