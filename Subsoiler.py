import math


class SubsoilerCalculator:
    """
    深松机耕深计算器
    公式: D = (L1 + Lt) * sin(α - β) - Ht * cos(α - β)
    其中:
        α: 机具（下拉杆/机具架）当前角度（度）
        β: 车身当前角度（度）
        α0, β0: 初始角度（用于归零）
    """

    def __init__(self, L1, Lt, Ht, install_angle=0.0, calibration=1.0, zero_offset=0.0):
        """
        参数:
            L1: 下拉杆长度（铰点至机具悬挂点），单位 mm
            Lt: 悬挂点至铲尖水平距离，单位 mm
            Ht: 悬挂点至铲尖垂直距离（铲尖低于悬挂点为正），单位 mm
            install_angle: 安装角偏移（度），额外加到角度差上
            calibration: 线性校准系数
            zero_offset: 零点偏移（mm），最终结果加上该值
        """
        self.L1 = L1
        self.Lt = Lt
        self.Ht = Ht
        self.install_angle = install_angle
        self.calibration = calibration
        self.zero_offset = zero_offset

    def _raw_depth(self, alpha, beta, alpha0, beta0):
        delta_alpha = alpha - alpha0
        delta_beta = beta - beta0
        relative_angle = (delta_alpha - delta_beta) + self.install_angle
        rad = math.radians(relative_angle)
        return (self.L1 + self.Lt) * math.sin(rad) - self.Ht * math.cos(rad)

    def calculate(self, alpha, beta, alpha0, beta0):
        """
        计算耕深（毫米）
        alpha, beta, alpha0, beta0 单位：度
        返回：耕深（mm）
        """
        raw = self._raw_depth(alpha, beta, alpha0, beta0)
        calibrated = raw * self.calibration + self.zero_offset
        return int(max(0, min(1000, calibrated)))

    def calibrate_zero(self, alpha0_deg, beta0_deg):
        """
        #根据给定的初始角度，自动设置 zero_offset，使该状态下计算出的深度为 0。
        #alpha0_deg, beta0_deg: 初始时机具和车身的绝对角度（度）
        """
        raw0 = self._raw_depth(alpha0_deg, beta0_deg, alpha0_deg, beta0_deg)
        self.zero_offset = -raw0 * self.calibration
