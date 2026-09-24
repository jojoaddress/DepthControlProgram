import math
import numpy as np
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any

plt.rcParams["font.sans-serif"] = [
    "SimHei",
    "Microsoft YaHei",
    "WenQuanYi Micro Hei",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False  # 解决负号显示问题


# ======================= 1. 地形与车身模型 =======================
class TerrainModel:
    """生成地形剖面并计算车身俯仰角（假设拖拉机始终贴合地面）"""

    def __init__(
        self,
        length: float = 100.0,
        resolution: float = 0.2,
        wave_amplitude: float = 0.05,
        wave_period: float = 10.0,
        random_roughness: float = 0.01,
    ):
        """
        :param length:       作业距离（米）
        :param resolution:   采样间距（米）
        :param wave_amplitude: 正弦波幅值（米）
        :param wave_period:   正弦波周期（米）
        :param random_roughness: 随机噪声标准差（米）
        """
        self.x = np.arange(0, length, resolution)
        # 地形高度 = 正弦波 + 随机噪声
        self.elevation = wave_amplitude * np.sin(
            2 * np.pi * self.x / wave_period
        ) + np.random.normal(0, random_roughness, len(self.x))
        # 计算坡度（弧度）-> 车身俯仰角（度）
        dx = resolution
        dy = np.gradient(self.elevation, dx)
        self.slope_rad = np.arctan(dy)
        self.beta_deg = np.degrees(self.slope_rad)  # 车身俯仰角 β

    def get_beta(self, index: int) -> float:
        return self.beta_deg[index]

    def get_speed(
        self,
        index: int,
        base_speed: float = 2.0,
        slope_coeff: float = 0.1,
        random_std: float = 0.2,
    ) -> float:
        """
        根据地形坡度模拟车速变化。
        :param base_speed:   基础车速 (m/s)
        :param slope_coeff:  坡度影响系数 (m/s per degree) 上坡减速，下坡加速
        :param random_std:   随机波动标准差 (m/s)
        :return: 瞬时速度 (m/s)
        """
        slope_deg = self.beta_deg[index]  # 车身俯仰角 = 地形坡度
        # 坡度影响：上坡（正坡度）减速，下坡加速
        speed = base_speed - slope_coeff * slope_deg
        # 添加随机波动
        speed += np.random.normal(0, random_std)
        # 限制合理范围 0.5 ~ 3.5 m/s
        return np.clip(speed, 0.5, 3.5)


# ======================= 2. 悬挂控制器（模拟机具调节）=======================
class SuspensionController:
    """简单比例控制器，使耕深跟踪目标值，输出机具相对于车身的转角 delta = α - β"""

    def __init__(self, Kp: float = 0.5, target_depth_mm: float = 200.0):
        self.Kp = Kp
        self.target_depth = target_depth_mm
        self.integral = 0.0

    def compute_delta(self, current_depth_mm: float, dt: float) -> float:
        """根据深度误差计算需要的相对角度变化（度）"""
        error = self.target_depth - current_depth_mm
        # 比例控制，输出角度调整量（度），限制范围防止过大
        delta_change = self.Kp * error
        delta_change = np.clip(delta_change, -5.0, 5.0)
        # 这里简化：返回调整后的相对角度（实际应累加，但为简单直接返回）
        # 更真实的模型需要状态保持，但为仿真合理，我们直接返回比例结果
        return delta_change


# ======================= 3. 机具计算器 =======================
# 导入外部机具类
from Subsoiler import SubsoilerCalculator
from RotaryTiller import RotaryTillerCalculator
from Plough import PloughCalculator


# ======================= 4. 仿真与验证引擎 =======================
class SimulationValidator:
    def __init__(
        self,
        implement: Any,  # 使用外部类，不再限制为 ImplementCalculator
        terrain: TerrainModel,
        alpha0_deg: float = 0.0,
        beta0_deg: float = 0.0,
        target_depth_mm: float = 200.0,
        add_sensor_noise_std: float = 0.5,
    ):
        self.impl = implement
        self.terrain = terrain
        self.alpha0 = alpha0_deg
        self.beta0 = beta0_deg
        self.target_depth = target_depth_mm
        self.noise_std = add_sensor_noise_std

    def run_simulation(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        n = len(self.terrain.x)
        beta_true = self.terrain.beta_deg
        x = self.terrain.x
        dx = x[1] - x[0]  # 固定空间步长 (m)
        # 初始相对角 gamma（使得初始深度为0，因为已标定）
        gamma = 0.0
        gamma_history = [gamma]
        alpha_true = np.zeros(n)
        alpha_true[0] = beta_true[0] + gamma

        # 控制参数（针对深松机调优）
        Kp = 0.015  # 增益 (deg/mm)
        max_rate = 5.0  # 最大相对角变化速率 (deg/s)
        tau = 0.1  # 液压系统时间常数 (s)

        for i in range(1, n):
            # 当前瞬时速度 (m/s)
            v = self.terrain.get_speed(
                i, base_speed=2.0, slope_coeff=0.08, random_std=0.15
            )
            dt = dx / v  # 时间步长 (s)
            if dt <= 0:
                dt = 0.05  # 安全保护

            # 上一时刻深度
            depth_prev = self.impl.calculate(
                alpha_true[i - 1], beta_true[i - 1], self.alpha0, self.beta0
            )
            error = self.target_depth - depth_prev

            # 期望的 gamma 变化率 (deg/s)
            delta_gamma_rate = Kp * error
            # 速率限制 (deg/s)
            delta_gamma_rate = np.clip(delta_gamma_rate, -max_rate, max_rate)
            # 角度变化量 (deg)
            delta_gamma_cmd = delta_gamma_rate * dt
            gamma_cmd = gamma + delta_gamma_cmd

            # 一阶滞后模拟液压响应 (时间常数 tau)
            gamma += (gamma_cmd - gamma) * (dt / tau)
            gamma_history.append(gamma)

            # 计算机具绝对角
            alpha_true[i] = beta_true[i] + gamma
            # 添加执行器噪声
            alpha_true[i] += np.random.normal(0, 0.05)

        # 传感器噪声（含车速影响可忽略）
        beta_meas = beta_true + np.random.normal(0, 0.1, n)
        alpha_meas = alpha_true + np.random.normal(0, 0.1, n)

        depth_calc = np.array(
            [
                self.impl.calculate(
                    alpha_meas[i], beta_meas[i], self.alpha0, self.beta0
                )
                for i in range(n)
            ]
        )
        return x, beta_meas, alpha_meas, depth_calc, gamma_history

    def evaluate(self, plot: bool = True) -> Dict[str, float]:
        x, beta, alpha, depth, gamma_history = self.run_simulation()
        error = depth - self.target_depth
        mae = np.mean(np.abs(error))
        rmse = np.sqrt(np.mean(error**2))
        max_err = np.max(np.abs(error))
        print(f"\n仿真评估结果（目标深度 = {self.target_depth} mm）")
        print(f"  平均绝对误差 MAE: {mae:.2f} mm")
        print(f"  均方根误差 RMSE: {rmse:.2f} mm")
        print(f"  最大绝对误差: {max_err:.2f} mm")
        # 特征长度打印已移除，因为外部类无此方法
        if plot:
            fig, axs = plt.subplots(4, 1, figsize=(12, 10))

            axs[0].plot(x, beta, label="车身俯仰角 β (含噪声)")
            axs[0].plot(x, alpha, label="机具角 α (含噪声)")
            axs[0].set_ylabel("角度 (度)")
            axs[0].legend()
            axs[0].grid(True)

            axs[1].plot(x, depth, label="计算耕深")
            axs[1].axhline(
                self.target_depth, color="r", linestyle="--", label="目标深度"
            )
            axs[1].set_ylabel("耕深 (mm)")
            axs[1].legend()
            axs[1].grid(True)

            axs[2].plot(x, error, label="误差")
            axs[2].set_xlabel("距离 (m)")
            axs[2].set_ylabel("误差 (mm)")
            axs[2].legend()
            axs[2].grid(True)

            axs[3].plot(x, gamma_history, label="相对角 γ (deg)")
            axs[3].set_xlabel("距离 (m)")
            axs[3].set_ylabel("γ (deg)")
            axs[3].legend()
            axs[3].grid(True)

            plt.tight_layout()
            plt.show()
        return {"MAE": mae, "RMSE": rmse, "MaxError": max_err}


# ======================= 5. 主测试函数 =======================
def main():
    np.random.seed(42)
    terrain = TerrainModel(
        length=100.0,
        resolution=0.2,
        wave_amplitude=0.08,
        wave_period=15.0,
        random_roughness=0.005,
    )

    subsoiler = SubsoilerCalculator(L1=1280, Lt=680, Ht=450, install_angle=0.0)
    subsoiler.calibrate_zero(0.0, 0.0)  # 零点标定
    implements = {
        "深松机": subsoiler,
        # "旋耕机": RotaryTillerCalculator(Larm=800, R=250, install_angle=0.0),
        # "翻转犁": PloughCalculator(Lbeam=1200, theta_install=15.0, install_angle=0.0),
    }

    for name, impl in implements.items():
        print(f"\n{'='*50}\n正在验证: {name}\n{'='*50}")
        validator = SimulationValidator(
            implement=impl,
            terrain=terrain,
            alpha0_deg=0.0,
            beta0_deg=0.0,
            target_depth_mm=-200.0,
            add_sensor_noise_std=0.3,
        )
        metrics = validator.evaluate(plot=True)


if __name__ == "__main__":
    main()
