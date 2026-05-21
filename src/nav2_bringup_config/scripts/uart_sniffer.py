#!/usr/bin/env python3
"""
uart_sniffer.py

通过 UART 抓包验证 cmd_vel_to_uart 节点发送的协议帧是否正确。
用法：
    python3 uart_sniffer.py --device /dev/ttyUSB1 --baudrate 115200
"""

import argparse
import serial
import struct
import sys


def main():
    parser = argparse.ArgumentParser(description="UART protocol sniffer for cmd_vel_to_uart")
    parser.add_argument("--device", default="/dev/ttyUSB1", help="Serial device (default: /dev/ttyUSB1)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.device, args.baudrate, timeout=0.1)
    except serial.SerialException as e:
        print(f"Failed to open {args.device}: {e}")
        sys.exit(1)

    print(f"Listening on {args.device} @ {args.baudrate}...")
    print("Frame format: [0xAA][0x55][f1][f2][f3][checksum][0x0D]")
    print("-" * 50)

    while True:
        try:
            raw = ser.read(16)
            if len(raw) != 16:
                continue

            # 检查帧头帧尾
            if raw[0] != 0xAA or raw[1] != 0x55 or raw[15] != 0x0D:
                print(f"  [SKIP] Bad frame header/tail: {raw.hex()}")
                continue

            # 校验和：前14字节异或
            checksum = 0
            for i in range(14):
                checksum ^= raw[i]

            if checksum != raw[14]:
                print(f"  [ERR ] Checksum mismatch: got {raw[14]:02X}, expected {checksum:02X}")
                print(f"        Raw: {raw.hex()}")
                continue

            # 解析三个 float
            f1, f2, f3 = struct.unpack('<fff', raw[2:14])

            hex_str = ' '.join(f'{b:02x}' for b in raw)
            print(f"  [OK  ] vx={f1:+.3f} m/s, vy={f2:+.3f} m/s, omega={f3:+.3f} rad/s")
            print(f"        Hex: {hex_str}")

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            break

    ser.close()


if __name__ == '__main__':
    main()
