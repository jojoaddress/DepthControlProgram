import device_model
import multiprocessing
import time
from datetime import datetime
import serial
from serial import SerialException
from enum import Enum, auto
import statistics
import json
import os
import requests
import threading
import queue

from Subsoiler import SubsoilerCalculator
from RotaryTiller import RotaryTillerCalculator
from Plough import PloughCalculator
from ForwardDepthModel import MultiToolHeightModel  # 正向深度模型容器

# 机具计算器注册表
CALCULATOR_REGISTRY = {
    "subsoiler": SubsoilerCalculator,
    "rotary_tiller": RotaryTillerCalculator,
    "plough": PloughCalculator,
}

# 完整协议相关常量
DEVICE_ADDR = 0x50  # 设备地址
FUNC_CODE = 0x06  # 功能码
PDA_START_BYTE = 0xA5  # PDA起始标志
PDA_END_BYTE = 0xAA  # PDA结束标志

# PDA指令定义
PDA_CMD_REQ_DEPTH_ONCE = 0x10  # 请求单次耕深数据（响应一次耕深+稳定性）
PDA_CMD_START_STREAM = (
    0x11  # 开始连续上报，未带数据则使用默认间隔1秒（1字节），范围[0.1,10]秒
)
PDA_CMD_STOP_STREAM = 0x12  # 停止连续上报
PDA_CMD_ZERO_CALIBRATE = 0x13  # 零点校准（铲尖刚好触地时下发）

PDA_CMD_TOOL_TYPE = 0x21  # 机具类别
PDA_CMD_SENSOR_STATUS = 0x22  # 传感器状态
PDA_CMD_DEPTH = 0x23  # 耕深
PDA_CMD_DEPTH_STABILITY = 0x24  # 当前耕深和深度稳定性
PDA_CMD_SUSPENSION_HEIGHT = 0x25  # 上发三点悬挂高度（2字节，%）

PDA_CMD_SET_TARGET_DEPTH = 0x30  # 设置目标耕深（3字节，mm）
PDA_CMD_SET_ACTUAL_HEIGHT = 0x31  # 下发实际三点悬挂高度（2字节，%）
PDA_CMD_SET_SPEED = 0x32  # 下发车辆作业速度（2字节，0.01 km/h，即放大100倍）


# 机具类型定义
TOOL_TYPE_ROTARY_TILLER = 0x01  # 旋耕机
TOOL_TYPE_SUBSOILER = 0x02  # 深松机
TOOL_TYPE_PLOUGH = 0x03  # 翻转犁

# 传感器状态定义
SENSOR_STATUS_NORMAL = 0x01  # 正常工作
SENSOR_STATUS_ERROR = 0x02  # 工作异常

# CRC16表格（与device_model中相同）
auchCRCHi = [
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
]

auchCRCLo = [
    0x00,
    0xC0,
    0xC1,
    0x01,
    0xC3,
    0x03,
    0x02,
    0xC2,
    0xC6,
    0x06,
    0x07,
    0xC7,
    0x05,
    0xC5,
    0xC4,
    0x04,
    0xCC,
    0x0C,
    0x0D,
    0xCD,
    0x0F,
    0xCF,
    0xCE,
    0x0E,
    0x0A,
    0xCA,
    0xCB,
    0x0B,
    0xC9,
    0x09,
    0x08,
    0xC8,
    0xD8,
    0x18,
    0x19,
    0xD9,
    0x1B,
    0xDB,
    0xDA,
    0x1A,
    0x1E,
    0xDE,
    0xDF,
    0x1F,
    0xDD,
    0x1D,
    0x1C,
    0xDC,
    0x14,
    0xD4,
    0xD5,
    0x15,
    0xD7,
    0x17,
    0x16,
    0xD6,
    0xD2,
    0x12,
    0x13,
    0xD3,
    0x11,
    0xD1,
    0xD0,
    0x10,
    0xF0,
    0x30,
    0x31,
    0xF1,
    0x33,
    0xF3,
    0xF2,
    0x32,
    0x36,
    0xF6,
    0xF7,
    0x37,
    0xF5,
    0x35,
    0x34,
    0xF4,
    0x3C,
    0xFC,
    0xFD,
    0x3D,
    0xFF,
    0x3F,
    0x3E,
    0xFE,
    0xFA,
    0x3A,
    0x3B,
    0xFB,
    0x39,
    0xF9,
    0xF8,
    0x38,
    0x28,
    0xE8,
    0xE9,
    0x29,
    0xEB,
    0x2B,
    0x2A,
    0xEA,
    0xEE,
    0x2E,
    0x2F,
    0xEF,
    0x2D,
    0xED,
    0xEC,
    0x2C,
    0xE4,
    0x24,
    0x25,
    0xE5,
    0x27,
    0xE7,
    0xE6,
    0x26,
    0x22,
    0xE2,
    0xE3,
    0x23,
    0xE1,
    0x21,
    0x20,
    0xE0,
    0xA0,
    0x60,
    0x61,
    0xA1,
    0x63,
    0xA3,
    0xA2,
    0x62,
    0x66,
    0xA6,
    0xA7,
    0x67,
    0xA5,
    0x65,
    0x64,
    0xA4,
    0x6C,
    0xAC,
    0xAD,
    0x6D,
    0xAF,
    0x6F,
    0x6E,
    0xAE,
    0xAA,
    0x6A,
    0x6B,
    0xAB,
    0x69,
    0xA9,
    0xA8,
    0x68,
    0x78,
    0xB8,
    0xB9,
    0x79,
    0xBB,
    0x7B,
    0x7A,
    0xBA,
    0xBE,
    0x7E,
    0x7F,
    0xBF,
    0x7D,
    0xBD,
    0xBC,
    0x7C,
    0xB4,
    0x74,
    0x75,
    0xB5,
    0x77,
    0xB7,
    0xB6,
    0x76,
    0x72,
    0xB2,
    0xB3,
    0x73,
    0xB1,
    0x71,
    0x70,
    0xB0,
    0x50,
    0x90,
    0x91,
    0x51,
    0x93,
    0x53,
    0x52,
    0x92,
    0x96,
    0x56,
    0x57,
    0x97,
    0x55,
    0x95,
    0x94,
    0x54,
    0x9C,
    0x5C,
    0x5D,
    0x9D,
    0x5F,
    0x9F,
    0x9E,
    0x5E,
    0x5A,
    0x9A,
    0x9B,
    0x5B,
    0x99,
    0x59,
    0x58,
    0x98,
    0x88,
    0x48,
    0x49,
    0x89,
    0x4B,
    0x8B,
    0x8A,
    0x4A,
    0x4E,
    0x8E,
    0x8F,
    0x4F,
    0x8D,
    0x4D,
    0x4C,
    0x8C,
    0x44,
    0x84,
    0x85,
    0x45,
    0x87,
    0x47,
    0x46,
    0x86,
    0x82,
    0x42,
    0x43,
    0x83,
    0x41,
    0x81,
    0x80,
    0x40,
]


def get_crc(datas, dlen):
    """计算CRC16校验值"""
    tempH = 0xFF  # 高 CRC 字节初始化
    tempL = 0xFF  # 低 CRC 字节初始化
    for i in range(0, dlen):
        tempIndex = (tempH ^ datas[i]) & 0xFF
        tempH = (tempL ^ auchCRCHi[tempIndex]) & 0xFF
        tempL = auchCRCLo[tempIndex]
    return (tempH << 8) | tempL


def load_config(config_path="config.json"):
    """加载配置文件"""
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"配置文件 {config_path} 不存在")
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def create_calculator(tool_type, tool_params):
    """工厂方法：根据机具类型创建对应的计算器实例"""
    if tool_type not in CALCULATOR_REGISTRY:
        raise ValueError(f"不支持的机具类型: {tool_type}")
    calculator_class = CALCULATOR_REGISTRY[tool_type]
    return calculator_class(**tool_params)


class PDASender:
    """PDA数据发送器（完整协议）"""

    def __init__(self, port_name, baud_rate=9600):
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.serial_port = None
        self.is_open = False

        # 当前状态
        self.current_tool_type = TOOL_TYPE_ROTARY_TILLER  # 默认旋耕机
        self.current_sensor_status = SENSOR_STATUS_NORMAL  # 默认正常工作

        self.rx_queue = queue.Queue()
        self.rx_thread = None
        self.rx_stop_event = threading.Event()

    def open_connection(self):
        """打开PDA串口连接（RS485）"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                write_timeout=1,
            )

            self.is_open = True
            print(f"RS485 PDA串口 {self.port_name} 已打开，波特率: {self.baud_rate}")
            if self.is_open:
                self.serial_port.reset_input_buffer()
                self.rx_stop_event.clear()
                self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
                self.rx_thread.start()
            return self.is_open
        except SerialException as e:
            print(f"打开RS485 PDA串口 {self.port_name} 失败: {e}")
            return False

    def close_connection(self):
        """关闭PDA串口连接"""
        self.rx_stop_event.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1)
        if self.serial_port and self.is_open:
            self.serial_port.close()
            self.is_open = False
            print("RS485 PDA串口已关闭")

    # PDA接收线程，持续监听RS485总线上的数据并解析完整协议数据包
    def _rx_loop(self):
        buffer = bytearray()
        first_packet = True
        while not self.rx_stop_event.is_set():
            if self.serial_port and self.serial_port.in_waiting:
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                buffer.extend(data)
                # 在接收循环中加入
                if first_packet and data:
                    print(f"[DBG] 首次收到数据: {data.hex().upper()}")
                    first_packet = False
                # 尝试解析完整数据包（最少10字节）
                while len(buffer) >= 10:
                    # 查找设备地址和功能码
                    if buffer[0] != DEVICE_ADDR or buffer[1] != FUNC_CODE:
                        buffer.pop(0)
                        continue
                    if len(buffer) < 3:
                        break
                    data_len = buffer[3] if len(buffer) > 3 else 0
                    total_len = 2 + data_len + 2  # addr+func + data_section + crc
                    if len(buffer) < total_len:
                        break
                    packet = buffer[:total_len]
                    buffer = buffer[total_len:]
                    self._parse_packet(packet)
            else:
                time.sleep(0.01)

    def _parse_packet(self, packet):
        # 整体CRC校验
        calc_crc = get_crc(packet[:-2], len(packet) - 2)
        recv_crc = (packet[-2] << 8) | packet[-1]

        if calc_crc != recv_crc:
            print("完整协议CRC校验失败")
            return
        # 提取数据区 [Start][Len][Cmd][Data...][PDACRC][End]
        data_section = packet[2:-2]
        if data_section[0] != PDA_START_BYTE or data_section[-1] != PDA_END_BYTE:
            print("PDA数据区格式错误")
            return
        # PDA CRC校验
        pda_crc_pos = len(data_section) - 3
        pda_calc = get_crc(data_section[:pda_crc_pos], pda_crc_pos)
        pda_recv = (data_section[pda_crc_pos] << 8) | data_section[pda_crc_pos + 1]
        if pda_calc != pda_recv:
            print("PDA数据区CRC校验失败")
            return
        cmd = data_section[2]
        real_data = data_section[3:pda_crc_pos]

        print(
            f"[RX] 收到指令 0x{cmd:02X}, 数据长度={len(real_data)}, 内容={real_data.hex().upper() if real_data else '无'}"
        )
        # 根据指令类型放入队列
        if cmd == PDA_CMD_REQ_DEPTH_ONCE:
            self.rx_queue.put(("req_once", None))
        elif cmd == PDA_CMD_START_STREAM:
            interval = 1.0
            if len(real_data) >= 1:
                interval = real_data[0] / 10.0
                interval = max(0.1, min(10.0, interval))
            self.rx_queue.put(("start_stream", interval))
        elif cmd == PDA_CMD_STOP_STREAM:
            self.rx_queue.put(("stop_stream", None))
        elif cmd == PDA_CMD_SET_TARGET_DEPTH:
            # 扩展：第3字节为控制使能（0=停止，非0=启动）
            if len(real_data) >= 2:
                depth = (real_data[0] << 8) | real_data[1]
                enable = True
                if len(real_data) >= 3:
                    enable = real_data[2] != 0
                self.rx_queue.put(("target_depth", (depth, enable)))

        elif cmd == PDA_CMD_SET_ACTUAL_HEIGHT:
            if len(real_data) >= 2:
                percent = real_data[1]  # 低字节为百分比
                percent = max(0, min(100, percent))
                self.rx_queue.put(("actual_height", percent))
                print(f"[RX] 设置实际悬挂高度: {percent}%")
        elif cmd == PDA_CMD_SET_SPEED:
            if len(real_data) >= 2:
                speed_raw = (real_data[0] << 8) | real_data[1]
                speed = speed_raw / 100.0
                self.rx_queue.put(("speed", speed))
        elif cmd == PDA_CMD_ZERO_CALIBRATE:
            self.rx_queue.put(("zero_calibrate", None))

    def get_command(self, block=False, timeout=None):
        try:
            return self.rx_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    def build_complete_packet(self, pda_data_bytes):
        """
        构建完整协议数据包
        格式: [设备地址(1)] + [功能码(1)] + [数据区(N)] + [CRC校验(2)]
        其中数据区 = [起始标志(1)] + [数据区长度(1)] + [指令(1)] + [真实数据(n)] + [CRC校验位(2)] + [结束标志(1)]
        """
        # 构建数据区部分（不包括CRC和End）
        data_section = [
            PDA_START_BYTE,  # 起始标志 (0xA5)
            0x00,  # 数据区长度 (先填0，后面计算)
        ] + pda_data_bytes  # PDA数据部分

        # 计算数据区长度（从起始标志到结束标志的所有字节数）
        # 数据区长度 = 起始标志 + 数据区长度字节 + PDA数据 + PDA CRC + 结束标志
        data_section_length = (
            len(data_section) + 3
        )  # +3 是因为要加上PDA CRC(2)和结束标志(1)
        data_section[1] = data_section_length

        # 计算PDA CRC（对数据区从起始标志到真实数据结束的部分进行计算）
        pda_crc = get_crc(data_section, len(data_section))
        pda_crc_high = (pda_crc >> 8) & 0xFF
        pda_crc_low = pda_crc & 0xFF

        # 完整数据区
        complete_data_section = data_section + [pda_crc_high, pda_crc_low, PDA_END_BYTE]

        # 构建完整协议（设备地址 + 功能码 + 数据区）
        complete_packet = [
            DEVICE_ADDR,  # 设备地址 (0x50)
            FUNC_CODE,  # 功能码 (0x06)
        ] + complete_data_section

        # 计算完整协议的CRC（对设备地址到数据区结束的部分进行计算）
        complete_crc = get_crc(complete_packet, len(complete_packet))
        complete_crc_high = (complete_crc >> 8) & 0xFF
        complete_crc_low = complete_crc & 0xFF

        # 完整协议数据包
        full_packet = complete_packet + [complete_crc_high, complete_crc_low]

        return bytes(full_packet)

    def send_complete_packet(self, packet):
        """发送完整协议数据包到RS485总线"""
        if not self.is_open or not self.serial_port:
            print("RS485 PDA串口未打开，无法发送数据")
            return False

        try:
            # 发送数据
            bytes_written = self.serial_port.write(packet)

            # 强制刷新输出缓冲区，确保数据立即发送
            self.serial_port.flush()

            print(
                f"已通过RS485发送完整协议数据包 ({bytes_written} 字节): {packet.hex().upper()}"
            )
            return True

        except Exception as e:
            print(f"通过RS485发送完整协议数据失败: {e}")
            return False

    def send_tool_type(self, tool_type):
        """发送机具类别（完整协议）"""
        # PDA数据部分 = [指令(1)] + [真实数据(1)]
        pda_data = [PDA_CMD_TOOL_TYPE, tool_type]
        packet = self.build_complete_packet(pda_data)
        return self.send_complete_packet(packet)

    def send_sensor_status(self, status):
        """发送传感器状态（完整协议）"""
        # PDA数据部分 = [指令(1)] + [真实数据(1)]
        pda_data = [PDA_CMD_SENSOR_STATUS, status]
        packet = self.build_complete_packet(pda_data)
        return self.send_complete_packet(packet)

    def send_depth_data(self, depth_mm):
        """
        发送耕深（毫米），直接使用毫米值发送
        """
        depth_mm = int(depth_mm)
        depth_mm = max(0, min(65535, depth_mm))
        depth_bytes = [(depth_mm >> 8) & 0xFF, depth_mm & 0xFF]
        pda_data = [PDA_CMD_DEPTH] + depth_bytes
        return self.send_complete_packet(self.build_complete_packet(pda_data))

    def send_depth_stability(self, depth_mm, stability_percent):
        """
        发送当前耕深和深度稳定性（指令 0x24）
        :param depth_mm: 耕深（毫米），整数 0~65535
        :param stability_percent: 稳定性百分比，整数 0~100
        """
        depth_mm = int(depth_mm)
        depth_mm = max(0, min(65535, depth_mm))
        stability_percent = max(0, min(100, stability_percent))
        depth_bytes = [(depth_mm >> 8) & 0xFF, depth_mm & 0xFF]
        pda_data = [PDA_CMD_DEPTH_STABILITY] + depth_bytes + [stability_percent]
        return self.send_complete_packet(self.build_complete_packet(pda_data))

    def send_height_data(self, height_percent):
        """发送三点悬挂高度百分比 (0-100)"""
        height_percent = max(0, min(100, int(height_percent)))
        # 协议：高字节0，低字节百分比
        pda_data = [PDA_CMD_SUSPENSION_HEIGHT, 0, height_percent]
        return self.send_complete_packet(self.build_complete_packet(pda_data))

    def print_packet_details(self, packet, command, data_bytes):
        """打印完整协议数据包详细信息"""
        print("\n完整协议数据包详细解析:")
        print("=" * 60)

        # 解析完整协议
        print("完整协议结构:")
        print(f"  设备地址: 0x{packet[0]:02X} (50)")
        print(f"  功能码: 0x{packet[1]:02X} (06)")

        # 解析数据区
        print("  数据区:")
        print(f"    起始标志: 0x{packet[2]:02X} (A5)")
        print(f"    数据区长度: 0x{packet[3]:02X} ({packet[3]} 字节)")
        print(f"    指令: 0x{packet[4]:02X}", end="")

        # 指令说明
        if command == PDA_CMD_TOOL_TYPE:
            print(" (21 - 机具类别)")
            tool_type = data_bytes[0]
            if tool_type == TOOL_TYPE_ROTARY_TILLER:
                print(f"      机具类型: 0x{tool_type:02X} (旋耕机)")
            elif tool_type == TOOL_TYPE_SUBSOILER:
                print(f"      机具类型: 0x{tool_type:02X} (深松机)")
            print(f"    PDA CRC: 0x{packet[6]:02X} 0x{packet[7]:02X}")
            print(f"    结束标志: 0x{packet[8]:02X} (AA)")
            print(f"  完整协议 CRC: 0x{packet[9]:02X} 0x{packet[10]:02X}")

        elif command == PDA_CMD_SENSOR_STATUS:
            print(" (22 - 传感器状态)")
            status = data_bytes[0]
            if status == SENSOR_STATUS_NORMAL:
                print(f"      传感器状态: 0x{status:02X} (正常工作)")
            elif status == SENSOR_STATUS_ERROR:
                print(f"      传感器状态: 0x{status:02X} (工作异常)")
            print(f"    PDA CRC: 0x{packet[6]:02X} 0x{packet[7]:02X}")
            print(f"    结束标志: 0x{packet[8]:02X} (AA)")
            print(f"  完整协议 CRC: 0x{packet[9]:02X} 0x{packet[10]:02X}")

        elif command == PDA_CMD_DEPTH:
            print(" (23 - 耕深)")
            depth_mm = (data_bytes[0] << 8) | data_bytes[1]
            print(
                f"      耕深值: {depth_mm}mm (0x{data_bytes[0]:02X} 0x{data_bytes[1]:02X})"
            )
            print(f"    PDA CRC: 0x{packet[7]:02X} 0x{packet[8]:02X}")
            print(f"    结束标志: 0x{packet[9]:02X} (AA)")
            print(f"  完整协议 CRC: 0x{packet[10]:02X} 0x{packet[11]:02X}")

        print(f"完整数据包: {packet.hex().upper()}")
        print("=" * 60)


def updateData(DeviceModel, shared_data):
    # 将设备数据更新到共享字典中
    shared_data[DeviceModel.deviceName] = DeviceModel.deviceData


def run_device(device_name, port, baud_rate, addr_list, shared_data):
    def device_callback(device):
        updateData(device, shared_data)

    device = device_model.DeviceModel(
        device_name, port, baud_rate, addr_list, device_callback
    )
    device.openDevice()
    device.startLoopRead()


def compute_stability(depth_history, min_samples=3):
    """
    根据深度历史列表计算稳定性百分比。
    depth_history: list of (timestamp, depth_mm)
    min_samples: 最少样本数
    返回: stability (int, 0~100)
    """
    if len(depth_history) < min_samples:
        return 100

    depths = [d for _, d in depth_history]

    # IQR 异常剔除
    sorted_depths = sorted(depths)
    n = len(sorted_depths)
    q1 = sorted_depths[int(0.25 * n)]
    q3 = sorted_depths[int(0.75 * n)]
    iqr = q3 - q1
    lower = q1 - 1.5 * iqr
    upper = q3 + 1.5 * iqr
    filtered = [d for d in depths if lower <= d <= upper]

    # 若剔除后样本太少，回退至原始数据
    if len(filtered) < min_samples:
        filtered = depths

    # 稳健统计：中位数和 MAD
    median = statistics.median(filtered)
    deviations = [abs(d - median) for d in filtered]
    mad = statistics.median(deviations)
    std_robust = 1.4826 * mad if mad > 0 else 0.0

    if median > 0:
        cv_robust = std_robust / median
        stability = max(0, min(100, 100 - cv_robust * 100))
    else:
        stability = 100.0

    return int(stability)


def init_logging(log_enabled, log_dir="."):
    if not log_enabled:
        return None, None
    os.makedirs(log_dir, exist_ok=True)
    raw_log_filename = f"raw_data-{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    model_log_filename = (
        f"model_train-{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    )
    raw_log_path = os.path.join(log_dir, raw_log_filename)
    model_log_path = os.path.join(log_dir, model_log_filename)
    try:
        raw_log_file = open(raw_log_path, "a", encoding="utf-8")
        model_log_file = open(model_log_path, "a", encoding="utf-8")
        # 更新后的日志列
        if os.path.getsize(raw_log_path) == 0:
            raw_log_file.write(
                "Timestamp,Beta(deg),Alpha(deg),Depth_mm,Stability_percent,"
                "PredDepth_mm,Speed_km/h,"
                "TargetDepth_mm,CmdHeight_percent,DepthError_mm,"
                "SoilHardness_MPa,SoilHardness_x_Depth\n"
            )

        if os.path.getsize(model_log_path) == 0:
            model_log_file.write(
                "Timestamp,UpdateIdx,ActualDepth_mm,PredDepth_mm,Error_mm\n"
            )
        print(f"原始数据日志: {raw_log_path}")
        print(f"模型收敛日志: {model_log_path}")
        return raw_log_file, model_log_file
    except Exception as e:
        print(f"创建日志文件失败: {e}")
        return None, None


def write_raw_log_entry(
    raw_log_file,
    beta,
    alpha,
    depth_mm,
    stability,
    pred_depth_mm,
    speed_kph,
    target_depth_mm,
    cmd_height_percent,
    depth_error_mm,
    soil_hardness,
):
    if raw_log_file is None:
        return
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # 格式化字段（处理 None / -1 等无效值）
    err_str = f"{depth_error_mm:.1f}" if depth_error_mm is not None else "NaN"
    pred_str = f"{pred_depth_mm:.1f}" if pred_depth_mm is not None else "NaN"
    speed_str = f"{speed_kph:.2f}" if speed_kph is not None else "NaN"

    soil_str = f"{soil_hardness:.2f}" if soil_hardness is not None else "NaN"

    soil_depth_str = (
        f"{soil_hardness * depth_mm:.2f}" if soil_hardness is not None else "NaN"
    )
    target_str = f"{target_depth_mm}" if target_depth_mm is not None else -1
    cmd_str = f"{cmd_height_percent:.1f}" if cmd_height_percent is not None else "NaN"
    stab_str = f"{stability}" if stability is not None else "NaN"

    # 写入文件
    raw_log_file.write(
        f"{timestamp},{beta:.3f},{alpha:.3f},{depth_mm:.2f},{stab_str},"
        f"{pred_str},{speed_str},"
        f"{target_str},{cmd_str},"
        f"{err_str},"
        f"{soil_str},{soil_depth_str}\n"
    )
    raw_log_file.flush()

    # 同步打印到控制台
    """ print(
        f"[LOG RAW] {timestamp} | β={beta:.2f}° α={alpha:.2f}° Depth={depth_mm:.1f}mm | "
        f"PredDepth={pred_str}mm | ActHeight={act_h_str}mm | "
        f"Target={target_str}mm | CmdHeight={cmd_str}mm | Err={err_str}mm"
    ) """


def write_model_log_entry(
    model_log_file, update_idx, actual_depth_mm, pred_depth_mm, error_mm
):
    if model_log_file is None:
        return
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # 写入文件
    model_log_file.write(
        f"{timestamp},{update_idx},{actual_depth_mm:.2f},{pred_depth_mm:.2f},{error_mm:.2f}\n"
    )
    model_log_file.flush()

    # 同步打印到控制台
    """ print(
        f"[LOG MODEL] {timestamp} | Update#{update_idx} | "
        f"ActualDepth={actual_depth_mm:.2f}mm PredDepth={pred_depth_mm:.2f}mm Error={error_mm:.2f}mm"
    ) """


def monitor_data(shared_data, pda_config, tool_config, monitor_config, device_names):
    pda_sender = PDASender(pda_config["port"], pda_config["baudrate"])
    if not pda_sender.open_connection():
        print("错误: 无法打开 PDA 串口，监控进程退出")
        return

    impl_name = device_names["implement"]
    veh_name = device_names["vehicle"]
    calculator = create_calculator(tool_config["type"], tool_config["params"])

    # 正向深度模型
    rls_forget = monitor_config.get("rls_forget_factor", 0.98)
    rls_delta = monitor_config.get("rls_delta", 100.0)
    rls_ridge = monitor_config.get("rls_ridge_penalty", 1e-4)
    height_model = MultiToolHeightModel(rls_forget, rls_delta, rls_ridge)
    model_save_dir = monitor_config.get("model_save_dir", "./models")
    model_save_interval = monitor_config.get("model_save_interval", 10)
    os.makedirs(model_save_dir, exist_ok=True)
    model_file = os.path.join(model_save_dir, "height_models.npz")

    if os.path.exists(model_file):
        try:
            height_model.load_models(model_file)
            print(f"已加载模型参数: {model_file}")
        except Exception as e:
            print(f"加载模型失败: {e}")
    else:
        print("未找到已保存的模型，将从零开始训练")

    tool_type_map = {
        "subsoiler": TOOL_TYPE_SUBSOILER,
        "rotary_tiller": TOOL_TYPE_ROTARY_TILLER,
        "plough": TOOL_TYPE_PLOUGH,
    }
    pda_sender.current_tool_type = tool_type_map.get(
        tool_config["type"], TOOL_TYPE_SUBSOILER
    )

    window_seconds = monitor_config.get("window_seconds", 10.0)
    min_samples = monitor_config.get("min_samples", 3)
    sleep_interval = monitor_config.get("sleep_interval", 1.0)
    sensor_tool_status_interval = monitor_config.get(
        "sensor_tool_status_interval", 0
    )  # 秒，0表示不启用定时发送
    log_enabled = monitor_config.get("log_enabled", True)
    log_dir = monitor_config.get("log_dir", "./logs")
    soil_hardness = monitor_config.get("soil_hardness_mpa", 2.0)
    default_speed = monitor_config.get("default_speed_kph", 5.0)
    height_send_interval = monitor_config.get("height_send_interval", 3)
    # 从配置文件读取控制增益，若未设置则默认0.8
    kp = monitor_config.get("control_kp", 0.8)
    min_updates_for_control = monitor_config.get("min_updates_for_control", 1000)

    # 全局模式控制器
    mode_ctx = ModeContext(
        height_model=height_model,
        tool_type=tool_config["type"],
        pda_sender=pda_sender,
        height_send_interval=height_send_interval,
        model_save_dir=model_save_dir,
        model_save_interval=model_save_interval,
        kp=kp,
        min_updates_for_control=min_updates_for_control,
    )

    # FedAvg
    fed_config = monitor_config.get("federated", {})
    fed_enabled = fed_config.get("enabled", True)
    if fed_enabled:
        server_url = fed_config.get("server_url", "127.0.0.1")
        upload_interval = fed_config.get("upload_interval", 20)
        download_interval = fed_config.get("download_interval", 300)
        client_id = fed_config.get("client_id", "unknown")

        # 启动时拉取全局模型
        if fed_config.get("download_on_start", True):
            try:
                resp = requests.get(
                    f"{server_url}/download/{tool_config['type']}", timeout=5
                )
                if resp.status_code == 200:
                    global_theta = resp.json()["theta"]
                    height_model.set_global_model(tool_config["type"], global_theta)
                    print(f"[联邦] 已加载全局模型，theta={global_theta}")
            except Exception as e:
                print(f"[联邦] 拉取全局模型失败: {e}")

        # 记录上次上传时的训练次数
        last_upload_count = 0

        # 启动后台线程定时拉取
        def periodic_pull():
            while True:
                time.sleep(download_interval)
                try:
                    resp = requests.get(
                        f"{server_url}/download/{tool_config['type']}", timeout=5
                    )
                    if resp.status_code == 200:
                        global_theta = resp.json()["theta"]
                        height_model.set_global_model(tool_config["type"], global_theta)
                        print("[联邦] 已更新为最新全局模型")
                except Exception as e:
                    print(f"[联邦] 定时拉取失败: {e}")

        if download_interval > 0:
            pull_thread = threading.Thread(target=periodic_pull, daemon=True)
            pull_thread.start()

    initial_alpha = None
    initial_beta = None
    initialized = False  # 是否已通过归零指令完成初始化
    initial_status_sent = False  # 初始状态是否已上报
    streaming = False
    stream_interval = 1.0
    last_stream_time = 0
    current_speed_kph = default_speed

    raw_log_file, model_log_file = init_logging(log_enabled, log_dir)
    last_actual_height = -1  # 待训练的悬挂高度
    last_cmd_height = -1  # 最后一次发送的控制高度
    last_sensor_status_time = 0  # 上次发送传感器状态的时间戳

    depth_history = []

    # 每轮循环中会重新计算的变量，先声明默认值
    current_alpha = None
    current_slope_x = None
    current_slope_y = None
    depth_mm = None
    slope_x = None
    slope_y = None
    stability = None
    can_calculate = False

    def send_depth_stability_report(depth_mm, stability):
        pda_sender.send_depth_data(depth_mm)
        pda_sender.send_depth_stability(depth_mm, stability)

    try:
        while True:
            current_alpha = None
            current_slope_x = None
            current_slope_y = None

            for device_name, data in shared_data.items():
                if (
                    data
                    and 0x50 in data
                    and "AngX" in data[0x50]
                    and "AngY" in data[0x50]
                    and "AngZ" in data[0x50]
                ):
                    ang_x = data[0x50].get("AngX")
                    ang_y = data[0x50].get("AngY")
                    ang_z = data[0x50].get("AngZ")
                    if device_name == impl_name:
                        current_alpha = ang_y
                    elif device_name == veh_name:
                        current_slope_x = ang_x  # 车辆横滚角 -> 横向坡度
                        current_slope_y = ang_y  # 车辆俯仰角 -> 纵向坡度

            current_time = time.time()

            # 一次性初始状态上报
            if (
                not initial_status_sent
                and current_alpha is not None
                and current_slope_y is not None
            ):
                pda_sender.send_tool_type(pda_sender.current_tool_type)
                time.sleep(0.1)
                pda_sender.send_sensor_status(pda_sender.current_sensor_status)
                initial_status_sent = True
                last_sensor_status_time = current_time  # 新增：记录初始发送时间
                print("已发送初始状态（机具类型 + 传感器正常）")

            # 统一指令处理
            while True:
                cmd = pda_sender.get_command(block=False)
                if cmd is None:
                    break
                typ, val = cmd

                # 指令处理映射表
                if typ == "zero_calibrate":
                    if current_alpha is not None and current_slope_y is not None:
                        initial_alpha = current_alpha
                        initial_beta = current_slope_y
                        initialized = True
                        calculator.calibrate_zero(current_alpha, current_slope_y)
                        depth_history.clear()
                        print(
                            f"归零完成 α0={initial_alpha:.2f}° β0={initial_beta:.2f}°"
                        )
                    else:
                        print("归零失败：无角度数据")

                elif typ == "target_depth":
                    depth, enable = val
                    mode_ctx.enter_control(depth, enable)
                    print(f"目标耕深: {depth} mm, 控制={'启用' if enable else '停止'}")

                elif typ == "speed":
                    current_speed_kph = val
                    print(f"作业速度更新: {current_speed_kph:.2f} km/h")

                elif typ == "start_stream":
                    streaming = True
                    stream_interval = val
                    last_stream_time = current_time
                    print(f"开始连续上报，间隔 {stream_interval} 秒")

                elif typ == "stop_stream":
                    streaming = False
                    print("停止连续上报")

                elif typ == "actual_height":
                    if not mode_ctx.is_control():
                        last_actual_height = val  # 仅记录，训练稍后统一处理
                        print(f"[记录] 实际悬挂高度: {val} %")

                elif typ == "req_once":
                    if can_calculate:
                        send_depth_stability_report(depth_mm, stability)
                        print("响应单次耕深请求")

            # 深度计算（仅当初始化）
            can_calculate = (
                initialized
                and current_alpha is not None
                and current_slope_y is not None
            )

            if can_calculate:
                depth_mm = calculator.calculate(
                    alpha=current_alpha,
                    beta=current_slope_y,
                    alpha0=initial_alpha,
                    beta0=initial_beta,
                )
                slope_x, slope_y = current_slope_x, current_slope_y

                # 稳定性计算
                depth_history.append((current_time, depth_mm))
                cutoff = current_time - window_seconds
                depth_history = [(t, d) for t, d in depth_history if t >= cutoff]
                stability = compute_stability(depth_history, min_samples)

            # 模型训练（处理缓存的真实悬挂高度）
            if can_calculate and last_actual_height != -1 and not mode_ctx.is_control():
                env_feats = (soil_hardness, current_speed_kph, slope_x, slope_y)
                result = mode_ctx.learn_from_actual_height(
                    last_actual_height, depth_mm, env_feats
                )
                if result is not None and model_log_file is not None:
                    pred_before, actual_d, err = result
                    model = mode_ctx.height_model.get_model(tool_config["type"])
                    write_model_log_entry(
                        model_log_file, model.training_count, actual_d, pred_before, err
                    )
                last_actual_height = -1  # 已处理
                # ========== 联邦上传逻辑 ==========
                if fed_enabled and result is not None:
                    model = mode_ctx.current_model
                    if model.training_count - last_upload_count >= upload_interval:
                        try:
                            theta, cnt = model.get_theta_with_count()
                            payload = {
                                "tool_type": tool_config["type"],
                                "theta": theta,
                                "training_count": cnt,
                                "client_id": client_id,
                            }
                            requests.post(
                                f"{server_url}/upload", json=payload, timeout=2
                            )
                            last_upload_count = model.training_count
                            print(f"[联邦] 已上传模型，训练次数={cnt}")
                        except Exception as e:
                            print(f"[联邦] 上传失败: {e}")

            # 控制模式指令下发
            if can_calculate:
                cmd_height = mode_ctx.try_issue_height_command(
                    (depth_mm, soil_hardness, current_speed_kph, slope_x, slope_y),
                    current_time,
                )
                if cmd_height is not None:
                    last_cmd_height = cmd_height

            # 连续上报
            if (
                can_calculate
                and streaming
                and (current_time - last_stream_time >= stream_interval)
            ):
                send_depth_stability_report(depth_mm, stability)
                last_stream_time = current_time

            # 定时发送传感器状态和机具类型
            if sensor_tool_status_interval > 0:
                if (
                    current_time - last_sensor_status_time
                    >= sensor_tool_status_interval
                ):
                    pda_sender.send_tool_type(pda_sender.current_tool_type)
                    time.sleep(0.1)
                    pda_sender.send_sensor_status(pda_sender.current_sensor_status)
                    last_sensor_status_time = current_time
                    print(f"[定时] 发送机具类型: {pda_sender.current_tool_type}")
                    print(f"[定时] 发送传感器状态: {pda_sender.current_sensor_status}")

            # 写入原始日志
            if can_calculate:
                pred_depth_for_log = (
                    mode_ctx.last_pred_depth if mode_ctx.last_pred_depth >= 0 else None
                )

                target_for_log = (
                    mode_ctx.target_depth_mm
                    if mode_ctx.target_depth_mm is not None
                    else -1
                )
                cmd_for_log = last_cmd_height if last_cmd_height != -1 else -1
                depth_error = (
                    (depth_mm - pred_depth_for_log)
                    if pred_depth_for_log is not None
                    else None
                )
                write_raw_log_entry(
                    raw_log_file,
                    current_slope_y,
                    current_alpha,
                    depth_mm,
                    stability,
                    pred_depth_for_log,
                    current_speed_kph,
                    target_for_log,
                    cmd_for_log,
                    depth_error,
                    soil_hardness,
                )
            else:
                if not initialized:
                    print("等待零点校准指令...")
            time.sleep(sleep_interval)

    finally:
        try:
            height_model.save_models(model_save_dir)
            print(f"最终模型已保存至 {model_save_dir}")
        except Exception as e:
            print(f"保存最终模型失败: {e}")
        if raw_log_file:
            raw_log_file.close()
        if model_log_file:
            model_log_file.close()
        pda_sender.close_connection()


class OperationMode(Enum):
    TRAINING = auto()
    CONTROL = auto()


class ModeContext:
    def __init__(
        self,
        height_model,
        tool_type,
        pda_sender,
        height_send_interval=3.0,
        model_save_dir="./models",
        model_save_interval=10,
        kp=0.8,
        min_updates_for_control=20,
    ):
        self.mode = OperationMode.TRAINING
        self.target_depth_mm = None
        self.height_model = height_model
        self.tool_type = tool_type
        self.pda_sender = pda_sender
        self.height_send_interval = height_send_interval
        self.last_cmd_time = 0
        self.model_save_dir = model_save_dir
        self.model_save_interval = model_save_interval
        self.kp = kp
        self.last_pred_depth = -1.0
        self.min_updates_for_control = min_updates_for_control

    def is_control(self):
        return self.mode == OperationMode.CONTROL

    def enter_control(self, depth_mm, enable):
        if enable:
            model = self.height_model.get_model(self.tool_type)
            if model.training_count < self.min_updates_for_control:
                print(
                    f"[警告] 模型训练不足 ({model.training_count}/{self.min_updates_for_control})，拒绝进入控制模式"
                )
                return
            self.mode = OperationMode.CONTROL
            self.target_depth_mm = depth_mm
            print(f"[模式] 进入控制模式，目标耕深={depth_mm}mm")
        else:
            self.mode = OperationMode.TRAINING
            self.target_depth_mm = None
            print("[模式] 返回训练模式")

    def learn_from_actual_height(
        self, actual_height_percent, actual_depth_mm, env_features
    ):
        if self.is_control():
            return None
        hardness, speed, slope_x, slope_y = env_features
        pred_before = self.height_model.predict(
            self.tool_type, actual_height_percent, hardness, speed, slope_x, slope_y
        )
        error = actual_depth_mm - pred_before
        self.height_model.update(
            self.tool_type,
            actual_height_percent,
            hardness,
            speed,
            slope_x,
            slope_y,
            actual_depth_mm,
        )
        # 更新计数器现在在模型内部自动完成
        model = self.height_model.get_model(self.tool_type)
        self.last_pred_depth = pred_before

        print(
            f"[训练] 更新#{model.training_count}: 高度={actual_height_percent}%, "
            f"几何深度={actual_depth_mm}mm, 预测深度={pred_before:.1f}mm, 误差={error:.1f}mm"
        )

        if model.training_count % self.model_save_interval == 0:
            try:
                self.height_model.save_models(self.model_save_dir)
            except Exception as e:
                print(f"保存模型失败: {e}")
        return pred_before, actual_depth_mm, error

    def try_issue_height_command(self, env_features, current_time):
        """控制模式：逆映射 + 误差闭环，返回指令高度(%) 或 None"""
        if not self.is_control():
            return None
        if current_time - self.last_cmd_time < self.height_send_interval:
            return None

        current_depth_mm, hardness, speed, slope_x, slope_y = env_features

        base_percent = self.height_model.inverse_predict(
            self.tool_type, self.target_depth_mm, hardness, speed, slope_x, slope_y
        )

        # 误差闭环，分母为深度对高度的导数
        error = self.target_depth_mm - current_depth_mm
        model = self.height_model.get_model(self.tool_type)
        denom = model.theta[1] + model.theta[6] * hardness  # ∂depth/∂(percent)
        if abs(denom) > 1e-4:
            adjusted_percent = base_percent + self.kp * error / denom
        else:
            adjusted_percent = base_percent

        cmd_percent = max(0, min(100, int(adjusted_percent)))
        self.pda_sender.send_height_data(cmd_percent)
        self.last_cmd_time = current_time

        # 预测深度用于日志
        pred_depth = self.height_model.predict(
            self.tool_type, cmd_percent, hardness, speed, slope_x, slope_y
        )
        self.last_pred_depth = pred_depth

        print(
            f"[控制] 上发悬挂高度: {cmd_percent}% (目标深度: {self.target_depth_mm}mm, "
            f"当前深度: {current_depth_mm}mm, 误差: {error:.1f}mm)"
        )
        return cmd_percent

    @property
    def current_model(self):
        return self.height_model.get_model(self.tool_type)


if __name__ == "__main__":
    config = load_config("config.json")
    manager = multiprocessing.Manager()
    shared_data = manager.dict()
    addr_list = [config["devices"]["implement"]["address"]]

    pda_config = config["devices"]["pda"]

    impl_config = config["devices"]["implement"]
    process_impl = multiprocessing.Process(
        target=run_device,
        args=(
            impl_config["name"],
            impl_config["port"],
            impl_config["baudrate"],
            addr_list,
            shared_data,
        ),
    )

    veh_config = config["devices"]["vehicle"]
    process_veh = multiprocessing.Process(
        target=run_device,
        args=(
            veh_config["name"],
            veh_config["port"],
            veh_config["baudrate"],
            addr_list,
            shared_data,
        ),
    )

    device_names = {"implement": impl_config["name"], "vehicle": veh_config["name"]}

    monitor_process = multiprocessing.Process(
        target=monitor_data,
        args=(shared_data, pda_config, config["tool"], config["monitor"], device_names),
    )

    process_impl.start()
    process_veh.start()
    monitor_process.start()

    try:
        process_impl.join()
        process_veh.join()
        monitor_process.join()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
