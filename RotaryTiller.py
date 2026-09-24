import math


class RotaryTillerCalculator:
    """
    旋耕机耕深计算器
    公式: D = Larm * sin(α - β) + R
    其中:
        α: 机具主梁当前角度（度）
        β: 车身当前角度（度）
        α0, β0: 初始角度（用于归零）
        Larm: 悬挂臂有效长度（下拉杆前铰点至刀辊轴心），单位 mm
        R: 刀辊半径（刀尖到轴心），单位 mm
    """

    def __init__(self, Larm, R, install_angle=0.0, calibration=1.0, zero_offset=0.0):
        """
        参数:
            Larm: 悬挂臂有效长度（mm）
            R: 刀辊半径（mm）
            install_angle: 安装角偏移（度），额外加到角度差上
            calibration: 线性校准系数
            zero_offset: 零点偏移（mm），最终结果减去该值
        """
        self.Larm = Larm
        self.R = R
        self.install_angle = install_angle
        self.calibration = calibration
        self.zero_offset = zero_offset

    def calculate(self, alpha, beta, alpha0, beta0):
        """
        计算耕深（毫米）
        alpha, beta, alpha0, beta0 单位：度
        返回：耕深（mm），非负
        """
        delta_alpha = alpha - alpha0
        delta_beta = beta - beta0
        relative_angle = (delta_alpha - delta_beta) + self.install_angle
        rad = math.radians(relative_angle)

        # 旋耕机专用公式
        raw_depth = self.Larm * math.sin(rad) + self.R
        calibrated = raw_depth * self.calibration - self.zero_offset
        return max(0, int(calibrated))
