#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import os
import struct
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse
from service_interfaces.msg import GetForceAct1, GetAngleAct1, GetTouchAct1, SetAngle1, SetForce1, SetSpeed1, GetTemp1 
from ament_index_python.packages import get_package_share_directory


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

# 定义电缸温度传感器数据地址和手指 ID
FORCE_SENSOR_RANGES = {
    0: (1618,),  # Pinky
    1: (1619,),  # Ring Finger
    2: (1620,),  # Middle Finger
    3: (1621,),  # Index Finger
    4: (1622,),  # Thumb Flexion
    5: (1623,),  # Thumb Abduction
}

# 定义触觉传感器数据起始地址（与 touch_data_ts 一致：每指读 25 个寄存器，法向力在 16-17，切向力在 20-21）
TOUCH_SENSOR_BASE_ADDR = {
    0: 3000,   # 小拇指 Pinky
    1: 3058,   # 无名指 Ring Finger
    2: 3116,   # 中指 Middle Finger
    3: 3174,   # 食指 Index Finger
    4: 3232,   # 大拇指 Thumb
}

# 定义设置力的地址和手指 ID
FORCE_SET_RANGES = {
    0: (1498,),  # Pinky
    1: (1500,),  # Ring Finger
    2: (1502,),  # Middle Finger
    3: (1504,),  # Index Finger
    4: (1506,),  # Thumb Flexion
    5: (1508,),  # Thumb Abduction
}

# 定义设置速度的地址和手指 ID
SPEED_SET_RANGES = {
    0: (1522,),  # Pinky
    1: (1524,),  # Ring Finger
    2: (1526,),  # Middle Finger
    3: (1528,),  # Index Finger
    4: (1530,),  # Thumb Flexion
    5: (1532,),  # Thumb Abduction
}

# 定义角度传感器数据地址和手指 ID
ANGLE_ACT_RANGES = {
    0: (1546,),  # Pinky
    1: (1548,),  # Ring Finger
    2: (1550,),  # Middle Finger
    3: (1552,),  # Index Finger
    4: (1554,),  # Thumb Flexion
    5: (1556,),  # Thumb Abduction
}

# 定义设置角度的地址和手指 ID
ANGLE_SET_RANGES = {
    0: (1486,),  # Pinky
    1: (1488,),  # Ring Finger
    2: (1490,),  # Middle Finger
    3: (1492,),  # Index Finger
    4: (1494,),  # Thumb Flexion
    5: (1496,),  # Thumb Abduction
}

# 创建字典映射手指 ID 到名称
FINGER_NAMES = {
    0: "Pinky Finger",
    1: "Ring Finger",
    2: "Middle Finger",
    3: "Index Finger",
    4: "Thumb Flexion",
    5: "Thumb Abduction",
}

class ModbusNode(Node):
    def __init__(self):
        super().__init__('sensor_data_publisher')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('spacename', 'inspire')
        self.declare_parameter('name', 'hand1')

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.publish_period = 0.1 if publish_rate <= 0.0 else 1.0 / publish_rate

        spacename = str(self.get_parameter('spacename').value).strip().strip('/')
        hand_name = str(self.get_parameter('name').value).strip().strip('/')
        topic_prefix_parts = [part for part in (spacename, hand_name) if part]
        self.topic_prefix = '/'.join(topic_prefix_parts)

        if self.topic_prefix:
            self.get_logger().info(
                f"topic 前缀: {self.topic_prefix}, 发布频率: {1.0 / self.publish_period:.2f} Hz"
            )
        else:
            self.get_logger().info(
                f"未配置 topic 前缀，仅使用基础 topic 名称，发布频率: {1.0 / self.publish_period:.2f} Hz"
            )

        def topic_name(base_name):
            return f"{self.topic_prefix}/{base_name}" if self.topic_prefix else base_name

        # 创建发布者
        self.force_publisher = self.create_publisher(GetForceAct1, topic_name('force_data'), 10)
        self.angle_publisher = self.create_publisher(GetAngleAct1, topic_name('angle_data'), 10)
        self.touch_publisher = self.create_publisher(GetTouchAct1, topic_name('touch_data'), 10)
        self.temp_publisher = self.create_publisher(GetTemp1, topic_name('temp_data'), 10)

        self.create_subscription(SetAngle1, topic_name('set_angle_data'), self.angle_callback, 10)
        self.create_subscription(SetForce1, topic_name('set_force_data'), self.force_callback, 10)
        self.create_subscription(SetSpeed1, topic_name('set_speed_data'), self.speed_callback, 10)

        # 创建 Modbus TCP 客户端
        self.modbus_client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
        if not self.modbus_client.connect():
            self.get_logger().error("无法连接到 Modbus 服务器，请检查 IP 和端口配置")
            return

        self.get_logger().info("Modbus 连接成功")

        # 启动数据读取线程
        self.read_thread = threading.Thread(target=self.data_reading_thread)
        self.read_thread.start()

    def read_signed_register(self, address):
        response = self.modbus_client.read_holding_registers(address=address, count=1)
        if isinstance(response, ExceptionResponse) or response.isError():
            self.get_logger().error(f"读取寄存器 {address} 失败: {response}")
            return 0
        else:
            value = response.registers[0]
            if value > 32767:
                value -= 65536
            return value

    def read_register_range(self, start_addr, end_addr):
        register_values = []
        for addr in range(start_addr, end_addr + 1, 125 * 2):
            current_count = min(125, (end_addr - addr) // 2 + 1)
            response = self.modbus_client.read_holding_registers(address=addr, count=current_count)

            if isinstance(response, ExceptionResponse) or response.isError():
                self.get_logger().error(f"读取寄存器 {addr} 失败: {response}")
                register_values.extend([0] * current_count)
            else:
                register_values.extend(response.registers)

        return register_values

    def _read_register_range_count(self, start_addr, count):
        """读取从 start_addr 开始的 count 个寄存器（触觉用，与 touch_data_ts 一致）。"""
        response = self.modbus_client.read_holding_registers(address=start_addr, count=count)
        if isinstance(response, ExceptionResponse) or response.isError():
            self.get_logger().error(f"读取寄存器 {start_addr} 失败: {response}")
            return None
        return response.registers

    @staticmethod
    def _read_float_from_bytes(registers, index):
        """从寄存器列表中 index 起 2 个寄存器解析为一个大端 float（与 touch_data_ts 一致）。"""
        byte0 = registers[index] & 0xFF
        byte1 = (registers[index] >> 8) & 0xFF
        byte2 = registers[index + 1] & 0xFF
        byte3 = (registers[index + 1] >> 8) & 0xFF
        combined = (byte3 << 24) | (byte2 << 16) | (byte1 << 8) | byte0
        return struct.unpack('!f', struct.pack('!I', combined))[0]

    def _read_finger_touch_data(self, base_addr):
        """
        读取单指触觉：法向力在 base+32（寄存器索引 16-17），切向力在 base+40（索引 20-21）。
        与 touch_data_ts 中 read_finger_data 一致。
        """
        register_values = self._read_register_range_count(base_addr, 25)
        if register_values is None or len(register_values) < 22:
            return float('nan'), float('nan')
        normal_force = self._read_float_from_bytes(register_values, 16)
        tangential_force = self._read_float_from_bytes(register_values, 20)
        return normal_force, tangential_force

    def read_touch_data(self):
        """按 touch_data_ts 方式读取五指法向力、切向力。"""
        finger_ids = []
        finger_names = []
        normal_forces = []
        tangential_forces = []
        for finger_id in sorted(TOUCH_SENSOR_BASE_ADDR.keys()):
            base_addr = TOUCH_SENSOR_BASE_ADDR[finger_id]
            nf, tf = self._read_finger_touch_data(base_addr)
            finger_ids.append(finger_id)
            finger_names.append(FINGER_NAMES.get(finger_id, "Unknown"))
            normal_forces.append(nf)
            tangential_forces.append(tf)
        return {
            'finger_ids': finger_ids,
            'finger_names': finger_names,
            'normal_forces': normal_forces,
            'tangential_forces': tangential_forces,
        }

    def read_temperature_data(self):
        temp_data = {}
        for finger_id in FORCE_SENSOR_RANGES.keys():
            address = FORCE_SENSOR_RANGES[finger_id][0]  
            temp_value = self.read_signed_register(address)  
            
            # 仅取低八位
            low_byte_value = temp_value & 0xFF
            temp_data[finger_id] = low_byte_value
            self.get_logger().info(f"Finger ID: {finger_id}, Temperature (Low Byte): {low_byte_value}")

        return temp_data

    def publish_data(self):
        start_time = time.time()

        if self.force_publisher.get_subscription_count() > 0:
            force_data_msg = GetForceAct1()
            force_data_msg.finger_ids = []
            force_data_msg.force_values = []
            force_data_msg.finger_names = []

            for finger_id, (start_addr,) in FORCE_SENSOR_RANGES.items():
                force_value = self.read_signed_register(start_addr)
                force_data_msg.finger_ids.append(finger_id)
                force_data_msg.force_values.append(force_value)
                force_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

            self.force_publisher.publish(force_data_msg)
            self.get_logger().info(f"已发布力数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

        if self.angle_publisher.get_subscription_count() > 0:
            angle_data_msg = GetAngleAct1()
            angle_data_msg.finger_ids = []
            angle_data_msg.angles = []
            angle_data_msg.finger_names = []

            for finger_id, (start_addr,) in ANGLE_ACT_RANGES.items():
                angle_value = self.read_signed_register(start_addr)
                angle_data_msg.finger_ids.append(finger_id)
                angle_data_msg.angles.append(angle_value)
                angle_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

            self.angle_publisher.publish(angle_data_msg)
            self.get_logger().info(f"已发布角度数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

        if self.touch_publisher.get_subscription_count() > 0:
            touch_data = self.read_touch_data()
            touch_data_msg = GetTouchAct1()
            touch_data_msg.finger_ids = touch_data['finger_ids']
            touch_data_msg.finger_names = touch_data['finger_names']
            touch_data_msg.normal_forces = touch_data['normal_forces']
            touch_data_msg.tangential_forces = touch_data['tangential_forces']

            self.touch_publisher.publish(touch_data_msg)
            self.get_logger().info(f"已发布触觉数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

        if self.temp_publisher.get_subscription_count() > 0:
            temp_data_msg = GetTemp1()
            temp_data_msg.finger_ids = []
            temp_data_msg.temp_values = []
            temp_data_msg.finger_names = []

            temperature_data = self.read_temperature_data()  # 读取电缸温度
            for finger_id, temp_value in temperature_data.items():
                temp_data_msg.finger_ids.append(finger_id)
                temp_data_msg.temp_values.append(temp_value)
                temp_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

            self.temp_publisher.publish(temp_data_msg)
            self.get_logger().info(f"已发布温度数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

    def angle_callback(self, msg):
        for finger_id, angle in zip(msg.finger_ids, msg.angles):
            if 0 <= angle <= 1000:
                address = ANGLE_SET_RANGES.get(finger_id, (None,))[0]
                if address is not None:
                    self.write_signed_register(address, angle)
                else:
                    self.get_logger().warn(f"未找到手指 ID {finger_id} 的地址")

    def force_callback(self, msg):
        for finger_id, force in zip(msg.finger_ids, msg.forces):
            if 0 <= force <= 3000:
                address = FORCE_SET_RANGES.get(finger_id, (None,))[0]
                if address is not None:
                    self.write_signed_register(address, force)
                else:
                    self.get_logger().warn(f"未找到手指 ID {finger_id} 的地址")

    def speed_callback(self, msg):
        for finger_id, speed in zip(msg.finger_ids, msg.speeds):
            if 0 <= speed <= 1000:
                address = SPEED_SET_RANGES.get(finger_id, (None,))[0]
                if address is not None:
                    self.write_signed_register(address, speed)
                else:
                    self.get_logger().warn(f"未找到手指 ID {finger_id} 的地址")

    def write_signed_register(self, address, value):
        if value < 0:
            value += 65536
        response = self.modbus_client.write_register(address, value)

        if isinstance(response, ExceptionResponse) or response.isError():
            self.get_logger().error(f"写入寄存器 {address} 失败: {response}")

    def data_reading_thread(self):
        while rclpy.ok():
            self.publish_data()
            time.sleep(self.publish_period)

def main(args=None):
    rclpy.init(args=args)

    modbus_node = ModbusNode()

    rclpy.spin(modbus_node)

    modbus_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

