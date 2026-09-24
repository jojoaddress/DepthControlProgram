import struct
import threading
import time
import serial
from protocol_constants import get_crc  # 直接复用工程已有 CRC16 算法


class VirtualSensorDevice:
    """虚拟倾角传感器设备（Modbus-RTU 从机模式）"""

    def __init__(self, port_name: str, baudrate: int = 9600, device_addr: int = 0x50):
        self.port_name = port_name
        self.baudrate = baudrate
        self.device_addr = device_addr

        # 实时姿态数据（度）
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.serial_port = None
        self._is_running = False
        self._lock = threading.Lock()

    def start(self) -> bool:
        """启动监听线程"""
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
            self._is_running = True
            threading.Thread(target=self._worker_loop, daemon=True).start()
            return True
        except Exception as e:
            print(f"[VirtualSensor] 打开端口 {self.port_name} 失败: {e}")
            return False

    def stop(self):
        """停止设备"""
        self._is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def update_attitude(self, pitch: float, roll: float = 0.0, yaw: float = 0.0):
        """线程安全地更新当前姿态角"""
        with self._lock:
            self.pitch = pitch
            self.roll = roll
            self.yaw = yaw

    def _pack_modbus_frame(self) -> bytes:
        """打包 24 字节 Modbus 响应数据（对标 device_model.processData 解包规则）[cite: 4]"""
        def deg_to_raw(deg: float) -> int:
            return max(-32768, min(32767, int(deg / 180.0 * 32768)))

        with self._lock:
            p, r, y = self.pitch, self.roll, self.yaw

        # 构造 24 字节数据体: Acc(6) + As(6) + H(6) + Ang(6)[cite: 4]
        payload = bytearray(18)  # 前 18 字节填 0[cite: 4]
        payload.extend(struct.pack(">h", deg_to_raw(r)))
        payload.extend(struct.pack(">h", deg_to_raw(p)))
        payload.extend(struct.pack(">h", deg_to_raw(y)))

        # 组装 Modbus-RTU 报文: Addr(1) + Func(1) + Len(1) + Data(24)[cite: 4]
        frame = bytearray([self.device_addr, 0x03, len(payload)]) + payload
        crc = get_crc(frame, len(frame))
        frame.append((crc >> 8) & 0xFF)
        frame.append(crc & 0xFF)
        return bytes(frame)

    def _worker_loop(self):
        """串口轮询处理循环"""
        while self._is_running:
            try:
                if self.serial_port.in_waiting >= 8:
                    request = self.serial_port.read(self.serial_port.in_waiting)
                    # 识别 0x50 地址与 0x03 读指令[cite: 4]
                    if len(request) >= 8 and request[0] == self.device_addr and request[1] == 0x03:
                        response = self._pack_modbus_frame()
                        self.serial_port.write(response)
            except Exception:
                pass
            time.sleep(0.01)