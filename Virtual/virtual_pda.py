import queue
import threading
import time
import serial
from protocol_constants import (
    get_crc, DEVICE_ADDR, FUNC_CODE, PDA_START_BYTE, PDA_END_BYTE,
    PDA_CMD_REQ_DEPTH_ONCE, PDA_CMD_START_STREAM, PDA_CMD_STOP_STREAM,
    PDA_CMD_ZERO_CALIBRATE, PDA_CMD_TOOL_TYPE, PDA_CMD_SENSOR_STATUS,
    PDA_CMD_DEPTH, PDA_CMD_DEPTH_STABILITY, PDA_CMD_SUSPENSION_HEIGHT,
    PDA_CMD_SET_TARGET_DEPTH, PDA_CMD_SET_ACTUAL_HEIGHT, PDA_CMD_SET_SPEED
)


class VirtualPDATerminal:
    """虚拟 PDA 终端设备（管理 RS485 总线交互）"""

    def __init__(self, port_name: str, baudrate: int = 9600):
        self.port_name = port_name
        self.baudrate = baudrate
        self.serial_port = None
        self.rx_queue = queue.Queue()
        self._is_running = False

    def start(self) -> bool:
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
            self._is_running = True
            threading.Thread(target=self._rx_loop, daemon=True).start()
            return True
        except Exception as e:
            print(f"[VirtualPDA] 打开端口 {self.port_name} 失败: {e}")
            return False

    def stop(self):
        self._is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def send_command(self, cmd_type: int, payload_bytes: list = None):
        """通用封装并发送 PDA 数据帧[cite: 6]"""
        if not self.serial_port or not self.serial_port.is_open:
            return

        payload = [cmd_type] + (payload_bytes if payload_bytes else [])
        data_section = [PDA_START_BYTE, len(payload) + 5] + payload
        pda_crc = get_crc(data_section, len(data_section))
        complete_data = data_section + [(pda_crc >> 8) & 0xFF, pda_crc & 0xFF, PDA_END_BYTE]

        full_frame = [DEVICE_ADDR, FUNC_CODE] + complete_data
        full_crc = get_crc(full_frame, len(full_frame))
        full_frame.extend([(full_crc >> 8) & 0xFF, full_crc & 0xFF])

        self.serial_port.write(bytes(full_frame))

    # 快捷业务封装
    def cmd_zero_calibrate(self):
        """发送触地零点标定 (0x13)[cite: 6]"""
        self.send_command(PDA_CMD_ZERO_CALIBRATE)

    def cmd_start_stream(self, interval_sec: float = 1.0):
        """发送开启数据流 (0x11)[cite: 6]"""
        val = int(interval_sec * 10)
        self.send_command(PDA_CMD_START_STREAM, [val])

    def cmd_stop_stream(self):
        """发送停止数据流 (0x12)[cite: 6]"""
        self.send_command(PDA_CMD_STOP_STREAM)

    def cmd_set_target_depth(self, depth_mm: int, enable: bool = True):
        """发送设置目标深度及闭环使能 (0x30)[cite: 6]"""
        self.send_command(PDA_CMD_SET_TARGET_DEPTH, [
            (depth_mm >> 8) & 0xFF, depth_mm & 0xFF, 1 if enable else 0
        ])

    def cmd_inject_actual_height(self, height_percent: int):
        """注入实际悬挂高度用于 RLS 在线更新 (0x31)[cite: 6]"""
        self.send_command(PDA_CMD_SET_ACTUAL_HEIGHT, [0, int(height_percent)])

    def _rx_loop(self):
        """接收并解析主程序下发指令的循环[cite: 6]"""
        buf = bytearray()
        while self._is_running:
            try:
                if self.serial_port.in_waiting:
                    buf.extend(self.serial_port.read(self.serial_port.in_waiting))
                    while len(buf) >= 10:
                        if buf[0] != DEVICE_ADDR or buf[1] != FUNC_CODE:
                            buf.pop(0)
                            continue
                        data_len = buf[3]
                        total_len = 2 + data_len + 2
                        if len(buf) < total_len:
                            break
                        packet = buf[:total_len]
                        buf = buf[total_len:]

                        # 校验 CRC[cite: 6]
                        if get_crc(packet[:-2], len(packet) - 2) == ((packet[-2] << 8) | packet[-1]):
                            cmd = packet[4]
                            data = packet[5:-3]
                            self._dispatch_pda_packet(cmd, data)
            except Exception:
                pass
            time.sleep(0.01)

    def _dispatch_pda_packet(self, cmd: int, data: bytearray):
        """将解析出的遥测数据放入线程队列"""
        if cmd == PDA_CMD_DEPTH and len(data) >= 2:
            self.rx_queue.put(("depth", (data[0] << 8) | data[1]))
        elif cmd == PDA_CMD_DEPTH_STABILITY and len(data) >= 3:
            depth = (data[0] << 8) | data[1]
            stability = data[2]
            self.rx_queue.put(("depth_stability", (depth, stability)))
        elif cmd == PDA_CMD_SUSPENSION_HEIGHT and len(data) >= 2:
            self.rx_queue.put(("cmd_height", data[1]))
        elif cmd == PDA_CMD_TOOL_TYPE and len(data) >= 1:
            self.rx_queue.put(("tool_type", data[0]))
        elif cmd == PDA_CMD_SENSOR_STATUS and len(data) >= 1:
            self.rx_queue.put(("sensor_status", data[0]))