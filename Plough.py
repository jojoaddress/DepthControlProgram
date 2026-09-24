import math


class PloughCalculator:
    """
    翻转犁耕深计算器
    公式: D = Lbeam * sin(α - β + θ_install)
    其中:
        α: 犁梁当前角度（度）
        β: 车身当前角度（度）
        α0, β0: 初始角度（用于归零）
        Lbeam: 犁梁长度（悬挂点至犁尖水平投影），单位 mm
        theta_install: 犁梁初始安装角（相对水平面，通常 10°~20°），单位 度
    """

    def __init__(
        self, Lbeam, theta_install, install_angle=0.0, calibration=1.0, zero_offset=0.0
    ):
        """
        参数:
            Lbeam: 犁梁长度（mm）
            theta_install: 犁梁初始安装角（度），即公式中的 θ_install
            install_angle: 额外安装角偏移（度），会叠加到角度差上（一般设为0，使用 theta_install 即可）
            calibration: 线性校准系数
            zero_offset: 零点偏移（mm），最终结果减去该值
        """
        self.Lbeam = Lbeam
        self.theta_install = theta_install
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
        relative_angle = (
            (delta_alpha - delta_beta) + self.theta_install + self.install_angle
        )
        rad = math.radians(relative_angle)

        # 翻转犁专用公式
        raw_depth = self.Lbeam * math.sin(rad)
        calibrated = raw_depth * self.calibration - self.zero_offset
        return max(0, int(calibrated))
