#!/usr/bin/env python3

import time
import os
import struct
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse
from ament_index_python.packages import get_package_share_directory
from service_interfaces.msg import GetTouchAct1


def load_modbus_config():
    """
    从安装的 share 目录中的 config.yaml 读取 Modbus IP 和端口。
    读取失败时回退到默认值。
    """
    default_ip = "192.168.11.210"
    default_port = 6000

    try:
        share_dir = get_package_share_directory("inspire_hand_modbus_ros2")
        config_path = os.path.join(share_dir, "config", "config.yaml")
        if not os.path.exists(config_path):
            return default_ip, default_port

        ip = default_ip
        port = default_port
        with open(config_path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.split("#", 1)[0].strip()
                if not line:
                    continue
                if "ip:" in line:
                    value = line.split("ip:", 1)[1].strip().strip("\"'")
                    if value:
                        ip = value
                elif "port:" in line:
                    value = line.split("port:", 1)[1].strip()
                    if value:
                        try:
                            port = int(value)
                        except ValueError:
                            pass
        return ip, port
    except Exception:
        return default_ip, default_port


MODBUS_IP, MODBUS_PORT = load_modbus_config()

# 触觉传感器：与 touch_data_ts 一致，每指一个基地址，读 25 个寄存器，法向力 16-17，切向力 20-21
TOUCH_SENSOR_BASE_ADDR = {
    0: 3000,   # 小拇指 Pinky
    1: 3058,   # 无名指 Ring Finger
    2: 3116,   # 中指 Middle Finger
    3: 3174,   # 食指 Index Finger
    4: 3232,   # 大拇指 Thumb
}

FINGER_NAMES = {
    0: "Pinky",
    1: "Ring Finger",
    2: "Middle Finger",
    3: "Index Finger",
    4: "Thumb",
}


def read_register_range_count(client, start_addr, count):
    """读取从 start_addr 开始的 count 个寄存器。"""
    response = client.read_holding_registers(address=start_addr, count=count)
    if isinstance(response, ExceptionResponse) or response.isError():
        return None
    return response.registers


def read_float_from_bytes(registers, index):
    """从寄存器列表 index 起 2 个寄存器解析为大端 float（与 touch_data_ts 一致）。"""
    byte0 = registers[index] & 0xFF
    byte1 = (registers[index] >> 8) & 0xFF
    byte2 = registers[index + 1] & 0xFF
    byte3 = (registers[index + 1] >> 8) & 0xFF
    combined = (byte3 << 24) | (byte2 << 16) | (byte1 << 8) | byte0
    return struct.unpack('!f', struct.pack('!I', combined))[0]


def read_finger_touch_data(client, base_addr):
    """读取单指触觉：法向力 16-17，切向力 20-21。"""
    regs = read_register_range_count(client, base_addr, 25)
    if regs is None or len(regs) < 22:
        return float('nan'), float('nan')
    normal_force = read_float_from_bytes(regs, 16)
    tangential_force = read_float_from_bytes(regs, 20)
    return normal_force, tangential_force


class HandControlPublisher(Node):
    def __init__(self):
        super().__init__("handcontrol_publisher")

        publish_rate = float(self.declare_parameter("publish_rate", 50.0).value)
        spacename = str(self.declare_parameter("spacename", "inspire").value).strip("/")
        hand_name = str(self.declare_parameter("name", "hand1").value).strip("/")
        topic_prefix = "/".join(part for part in (spacename, hand_name) if part)
        touch_topic = f"{topic_prefix}/touch_data" if topic_prefix else "touch_data"

        self.publisher_ = self.create_publisher(GetTouchAct1, touch_topic, 10)
        self.timer = self.create_timer(1.0 / max(0.1, publish_rate), self.publish_touch_data)
        self.modbus_client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)

        if not self.modbus_client.connect():
            self.get_logger().error("无法连接到 Modbus 服务器")
            raise RuntimeError("Modbus 连接失败")
        self.get_logger().info(
            f"Modbus 连接成功，发布话题: {touch_topic}，频率: {publish_rate:.2f} Hz"
        )

    def publish_touch_data(self):
        start_time = time.time()

        finger_ids = []
        finger_names = []
        normal_forces = []
        tangential_forces = []
        for finger_id in sorted(TOUCH_SENSOR_BASE_ADDR.keys()):
            base_addr = TOUCH_SENSOR_BASE_ADDR[finger_id]
            nf, tf = read_finger_touch_data(self.modbus_client, base_addr)
            finger_ids.append(finger_id)
            finger_names.append(FINGER_NAMES[finger_id])
            normal_forces.append(nf)
            tangential_forces.append(tf)

        msg = GetTouchAct1()
        msg.finger_ids = finger_ids
        msg.finger_names = finger_names
        msg.normal_forces = normal_forces
        msg.tangential_forces = tangential_forces
        self.publisher_.publish(msg)

        end_time = time.time()
        frequency = 1 / (end_time - start_time)
        self.get_logger().info(f"读取频率：{frequency:.2f} Hz")


def main(args=None):
    rclpy.init(args=args)
    publisher = HandControlPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info("手动停止节点")
    finally:
        publisher.modbus_client.close()
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
