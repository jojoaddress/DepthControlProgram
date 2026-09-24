from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt


class Pose3DViewWidget(QWidget):
    """
    3D 农机与悬挂机构可视化视口 (扩展预留)
    后续可将此类无缝替换为基于 QOpenGLWidget 或 pyqtgraph.opengl.GLViewWidget 的实现
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.status_label = QLabel("3D 渲染视口 (预留空间)\n当前姿态数据挂载已就绪")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("color: #888888; font-size: 14px; border: 1px dashed #555;")
        layout.addWidget(self.status_label)

    def update_pose(self, alpha: float, beta: float, current_depth: float, height_pct: float):
        """接收仿真状态并更新 3D 姿态矩阵"""
        # 后续接入 3D 模型旋转、平移的统一入口
        self.status_label.setText(
            f"3D 模型渲染接口\n"
            f"机身 β: {beta:.2f}° | 机具 α: {alpha:.2f}°\n"
            f"实时耕深: {current_depth:.1f} mm | 悬挂位移: {height_pct:.1f}%"
        )