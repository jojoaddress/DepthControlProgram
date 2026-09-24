import json
import sys
import time
from collections import deque

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QPushButton, QSlider, QSpinBox,
    QTabWidget, QTextEdit, QVBoxLayout, QWidget
)
import pyqtgraph as pg

from plant_simulator import PlantSimulator
from view_3d_widget import Pose3DViewWidget
from virtual_pda import VirtualPDATerminal
from virtual_sensor import VirtualSensorDevice


class TestBenchMainWindow(QMainWindow):
    def __init__(self, config_path: str = "config_test.json"):
        super().__init__()
        self.setWindowTitle("耕深智控系统综合测试台")
        self.resize(1200, 750)

        # 加载配置
        self.cfg = self._load_config(config_path)

        # 实例化业务实体
        p = self.cfg["serial_ports"]
        b = self.cfg["baudrate"]
        self.sensor_veh = VirtualSensorDevice(p["vehicle_sensor"], b)
        self.sensor_impl = VirtualSensorDevice(p["implement_sensor"], b)
        self.pda = VirtualPDATerminal(p["pda_bus"], b)

        sim_cfg = self.cfg.get("simulation", {})
        self.plant = PlantSimulator(
            tau=sim_cfg.get("hydraulic_tau", 0.8),
            angle_gain=sim_cfg.get("angle_gain", 0.4)
        )

        # 数据缓冲区
        self.history_len = 200
        self.time_buf = deque(maxlen=self.history_len)
        self.depth_buf = deque(maxlen=self.history_len)
        self.target_buf = deque(maxlen=self.history_len)
        self.height_buf = deque(maxlen=self.history_len)
        self.start_time = time.time()
        self.current_target_depth = 0
        self.latest_depth = 0.0

        self._init_ui()

        # 20Hz 刷新循环 (50ms)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._main_loop_step)
        self.timer.start(50)

    def _load_config(self, path: str) -> dict:
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {
                "serial_ports": {"vehicle_sensor": "COM17", "implement_sensor": "COM19", "pda_bus": "COM20"},
                "baudrate": 9600,
                "simulation": {"hydraulic_tau": 0.8, "angle_gain": 0.4}
            }

    def _init_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)

        # 左侧控制面板 (1/3 宽度)
        left_layout = QVBoxLayout()
        left_layout.addWidget(self._build_connection_group())
        left_layout.addWidget(self._build_attitude_group())
        left_layout.addWidget(self._build_pda_group())
        left_layout.addWidget(self._build_telemetry_group())
        left_layout.addStretch()
        layout.addLayout(left_layout, 1)

        # 右侧 Tab 面板：2D 波形与 3D 视口 (2/3 宽度)
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_plots_tab(), "2D 实时波形")
        self.view_3d = Pose3DViewWidget()
        self.tabs.addTab(self.view_3d, "3D 机构仿真")
        layout.addWidget(self.tabs, 2)

    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox("串口通信管理")
        l = QVBoxLayout(grp)
        self.btn_conn = QPushButton("启动虚拟硬件链路")
        self.btn_conn.clicked.connect(self._toggle_connection)
        l.addWidget(self.btn_conn)
        return grp

    def _build_attitude_group(self) -> QGroupBox:
        grp = QGroupBox("姿态与物理仿真")
        g = QGridLayout(grp)

        self.lbl_alpha = QLabel("手动机具 α: 0.0°")
        self.slider_alpha = QSlider(Qt.Orientation.Horizontal)
        self.slider_alpha.setRange(-450, 450)
        self.slider_alpha.setValue(0)

        self.lbl_beta = QLabel("车身俯仰 β: 0.0°")
        self.slider_beta = QSlider(Qt.Orientation.Horizontal)
        self.slider_beta.setRange(-300, 300)
        self.slider_beta.setValue(0)

        self.chk_closed_loop = QCheckBox("启用液压悬挂物理闭环")
        self.chk_closed_loop.stateChanged.connect(
            lambda s: setattr(self.plant, "closed_loop_enabled", bool(s))
        )

        g.addWidget(self.lbl_alpha, 0, 0)
        g.addWidget(self.slider_alpha, 0, 1)
        g.addWidget(self.lbl_beta, 1, 0)
        g.addWidget(self.slider_beta, 1, 1)
        g.addWidget(self.chk_closed_loop, 2, 0, 1, 2)
        return grp

    def _build_pda_group(self) -> QGroupBox:
        grp = QGroupBox("虚拟 PDA 控制台")
        g = QGridLayout(grp)

        btn_zero = QPushButton("零点校准 (0x13)")
        btn_zero.clicked.connect(self.pda.cmd_zero_calibrate)

        btn_stream = QPushButton("启动数据流 (0x11)")
        btn_stream.clicked.connect(lambda: self.pda.cmd_start_stream(1.0))

        self.spin_target = QSpinBox()
        self.spin_target.setRange(0, 600)
        self.spin_target.setValue(250)
        self.spin_target.setSuffix(" mm")
        btn_target = QPushButton("下发目标深度 (0x30)")
        btn_target.clicked.connect(self._send_target_depth)

        self.spin_train_h = QSpinBox()
        self.spin_train_h.setRange(0, 100)
        self.spin_train_h.setValue(50)
        self.spin_train_h.setSuffix(" %")
        btn_train_h = QPushButton("注入悬挂高度 (0x31)")
        btn_train_h.clicked.connect(
            lambda: self.pda.cmd_inject_actual_height(self.spin_train_h.value())
        )

        g.addWidget(btn_zero, 0, 0)
        g.addWidget(btn_stream, 0, 1)
        g.addWidget(self.spin_target, 1, 0)
        g.addWidget(btn_target, 1, 1)
        g.addWidget(self.spin_train_h, 2, 0)
        g.addWidget(btn_train_h, 2, 1)
        return grp

    def _build_telemetry_group(self) -> QGroupBox:
        grp = QGroupBox("系统状态遥测")
        g = QGridLayout(grp)
        self.lbl_rx_depth = QLabel("反馈耕深: -- mm")
        self.lbl_rx_stability = QLabel("稳定性: -- %")
        self.lbl_rx_cmd_h = QLabel("下发高度: -- %")
        g.addWidget(self.lbl_rx_depth, 0, 0)
        g.addWidget(self.lbl_rx_stability, 0, 1)
        g.addWidget(self.lbl_rx_cmd_h, 1, 0, 1, 2)
        return grp

    def _build_plots_tab(self) -> QWidget:
        widget = QWidget()
        l = QVBoxLayout(widget)
        self.plot_view = pg.GraphicsLayoutWidget()

        # 耕深追踪曲线
        self.p_depth = self.plot_view.addPlot(title="耕深追踪响应 (mm)")
        self.p_depth.showGrid(x=True, y=True)
        self.p_depth.addLegend()
        self.curve_actual = self.p_depth.plot(pen=pg.mkPen('c', width=2), name="主程序解算耕深")
        self.curve_target = self.p_depth.plot(pen=pg.mkPen('r', width=1.5, style=Qt.PenStyle.DashLine), name="目标深度")

        self.plot_view.nextRow()
        # 悬挂高度曲线
        self.p_height = self.plot_view.addPlot(title="悬挂高度 (%)")
        self.p_height.showGrid(x=True, y=True)
        self.curve_height = self.p_height.plot(pen=pg.mkPen('y', width=2), name="控制器输出高度")

        l.addWidget(self.plot_view)
        return widget

    def _toggle_connection(self):
        s1 = self.sensor_veh.start()
        s2 = self.sensor_impl.start()
        s3 = self.pda.start()
        if s1 and s2 and s3:
            self.btn_conn.setEnabled(False)
            self.btn_conn.setText("虚拟链路已连接")

    def _send_target_depth(self):
        val = self.spin_target.value()
        self.current_target_depth = val
        self.pda.cmd_set_target_depth(val, enable=True)

    def _main_loop_step(self):
        """20Hz 时钟步进处理"""
        dt = 0.05
        now = time.time() - self.start_time

        # 1. 获取滑块输入并推进一步动力学
        manual_a = self.slider_alpha.value() / 10.0
        manual_b = self.slider_beta.value() / 10.0
        self.lbl_alpha.setText(f"机具 α: {self.plant.impl_pitch:.1f}°")
        self.lbl_beta.setText(f"车身 β: {self.plant.veh_pitch:.1f}°")

        self.plant.step(dt, manual_a, manual_b)

        # 2. 将计算得到的最新姿态写入虚拟传感器
        self.sensor_veh.update_attitude(pitch=self.plant.veh_pitch, roll=self.plant.veh_roll)
        self.sensor_impl.update_attitude(pitch=self.plant.impl_pitch)

        # 3. 解析 PDA 接收队列
        while not self.pda.rx_queue.empty():
            msg_type, data = self.pda.rx_queue.get_nowait()
            if msg_type in ("depth", "depth_stability"):
                d = data if msg_type == "depth" else data[0]
                self.latest_depth = d
                self.lbl_rx_depth.setText(f"反馈耕深: {d} mm")
                if msg_type == "depth_stability":
                    self.lbl_rx_stability.setText(f"稳定性: {data[1]} %")

                self.time_buf.append(now)
                self.depth_buf.append(d)
                self.target_buf.append(self.current_target_depth)

            elif msg_type == "cmd_height":
                self.lbl_rx_cmd_h.setText(f"下发高度: {data} %")
                self.plant.set_command_height(data)  # 物理闭环联动！
                self.height_buf.append(data)

        # 4. 刷新曲线
        if self.time_buf:
            t_list = list(self.time_buf)
            self.curve_actual.setData(t_list, list(self.depth_buf))
            self.curve_target.setData(t_list, list(self.target_buf))
        if self.height_buf:
            t_h = list(self.time_buf)[-len(self.height_buf):]
            self.curve_height.setData(t_h, list(self.height_buf))

        # 5. 驱动 3D 视口更新
        self.view_3d.update_pose(
            alpha=self.plant.impl_pitch,
            beta=self.plant.veh_pitch,
            current_depth=self.latest_depth,
            height_pct=self.plant.current_height
        )

    def closeEvent(self, event):
        self.sensor_veh.stop()
        self.sensor_impl.stop()
        self.pda.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TestBenchMainWindow()
    win.show()
    sys.exit(app.exec())