# HeightModel.py
"""三点悬挂正向深度自适应模型（高度 -> 深度）
输入特征: [1, 悬挂高度, 土壤硬度, 速度, 坡度X, 坡度Y, 高度×硬度]
输出: 耕深 (mm)
采用递推最小二乘(RLS)在线更新，支持解析逆映射用于控制。
"""

import numpy as np


class ForwardDepthModel:
    """单个机具的深度正向模型（线性Ridge + RLS）"""

    def __init__(
        self,
        tool_type: str,
        forget_factor: float = 0.98,
        delta: float = 100.0,
        ridge_penalty: float = 1e-4,
    ):
        self.tool_type = tool_type
        self.lambda_ = forget_factor
        self.ridge_penalty = ridge_penalty

        # 特征维度：偏置 + 高度 + 硬度 + 速度 + 坡度X + 坡度Y + 高度×硬度
        self.dim = 7
        self.theta = np.zeros(self.dim)
        self.P = np.eye(self.dim) * delta

        self.training_count = 0  # 累计训练次数

    def _construct_features(
        self,
        height_percent: float,  # 百分比 0-100
        hardness_mpa: float,
        speed_kph: float,
        slope_x_deg: float,
        slope_y_deg: float,
    ) -> np.ndarray:
        """构造特征: [1, h, c, v, sx, sy, h*c]"""
        return np.array(
            [
                1.0,
                height_percent,
                hardness_mpa,
                speed_kph,
                slope_x_deg,
                slope_y_deg,
                height_percent * hardness_mpa,
            ]
        )

    def predict(
        self,
        height_percent: float,
        hardness_mpa: float,
        speed_kph: float,
        slope_x_deg: float,
        slope_y_deg: float,
    ) -> float:
        """预测耕深 (mm)"""
        x = self._construct_features(
            height_percent, hardness_mpa, speed_kph, slope_x_deg, slope_y_deg
        )
        depth = np.dot(self.theta, x)
        # 限幅到安全范围 [0, 600] mm
        return max(0.0, min(600.0, depth))

    def update(
        self,
        height_percent: float,
        hardness_mpa: float,
        speed_kph: float,
        slope_x_deg: float,
        slope_y_deg: float,
        actual_depth_mm: float,
    ):
        """用实际耕深更新模型（RLS在线学习）"""
        x = self._construct_features(
            height_percent, hardness_mpa, speed_kph, slope_x_deg, slope_y_deg
        )
        pred = np.dot(self.theta, x)
        error = actual_depth_mm - pred

        Px = self.P @ x
        denom = self.lambda_ + np.dot(x, Px)
        k = Px / denom

        self.theta += k * error

        I = np.eye(self.dim)
        self.P = (I - np.outer(k, x)) @ self.P / self.lambda_
        self.P += self.ridge_penalty * np.eye(self.dim)

        self.training_count += 1  # 每次更新递增

    def inverse_predict(
        self,
        target_depth_mm: float,
        hardness_mpa: float,
        speed_kph: float,
        slope_x_deg: float,
        slope_y_deg: float,
        default_height_percent: float = 50.0,
    ) -> float:
        """
        解析求解逆映射：给定目标深度返回需要的悬挂高度。
        模型：depth = θ0 + θ1*h + θ2*c + θ3*v + θ4*sx + θ5*sy + θ6*(h*c)
        整理得：h = (target - (θ0 + θ2*c + θ3*v + θ4*sx + θ5*sy)) / (θ1 + θ6*c)
        若分母过小或结果超出范围，返回默认高度或边界值。
        """
        c = hardness_mpa
        v = speed_kph
        sx = slope_x_deg
        sy = slope_y_deg

        const_part = (
            self.theta[0]
            + self.theta[2] * c
            + self.theta[3] * v
            + self.theta[4] * sx
            + self.theta[5] * sy
        )
        denom = self.theta[1] + self.theta[6] * c

        # 分母保护（期望正相关，即增加高度深度增加）
        if abs(denom) < 1e-4:
            # 模型未训练或不可靠，返回默认高度
            return default_height_percent

        h_pred = (target_depth_mm - const_part) / denom

        # 限幅在可执行范围（可根据实际液压行程调整）
        h_pred = max(0.0, min(100.0, h_pred))
        return h_pred

    def get_params(self):
        return self.theta.copy()

    def set_params(self, theta: np.ndarray):
        if theta.shape == (self.dim,):
            self.theta = theta.copy()

    def get_theta_with_count(self):
        return self.theta.tolist(), self.training_count

    def set_theta(self, theta_list):
        """直接用全局模型替换本地 theta，P 矩阵重置为初始值（简单策略）"""
        self.theta = np.array(theta_list)
        # 重置协方差矩阵，避免过度信任旧信息
        self.P = np.eye(self.dim) * (self.P[0][0] if self.P[0][0] != 0 else 100.0)


class MultiToolHeightModel:
    """管理多种机具的RLS正向深度模型容器"""

    def __init__(self, forget_factor=0.98, delta=100.0, ridge_penalty=1e-4):
        self.models = {}
        self.default_forget = forget_factor
        self.default_delta = delta
        self.default_ridge = ridge_penalty

    def get_model(self, tool_type: str):
        if tool_type not in self.models:
            self.models[tool_type] = ForwardDepthModel(
                tool_type, self.default_forget, self.default_delta, self.default_ridge
            )
        return self.models[tool_type]

    def predict(
        self,
        tool_type: str,
        height_percent: float,
        hardness_mpa: float,
        speed_kph: float,
        slope_x_deg: float,
        slope_y_deg: float,
    ) -> float:
        """预测耕深"""
        model = self.get_model(tool_type)
        return model.predict(
            height_percent, hardness_mpa, speed_kph, slope_x_deg, slope_y_deg
        )

    def update(
        self,
        tool_type: str,
        height_percent: float,
        hardness_mpa: float,
        speed_kph: float,
        slope_x_deg: float,
        slope_y_deg: float,
        actual_depth_mm: float,
    ):
        """在线更新模型"""
        model = self.get_model(tool_type)
        model.update(
            height_percent,
            hardness_mpa,
            speed_kph,
            slope_x_deg,
            slope_y_deg,
            actual_depth_mm,
        )

    def inverse_predict(
        self,
        tool_type: str,
        target_depth_mm: float,
        hardness_mpa: float,
        speed_kph: float,
        slope_x_deg: float,
        slope_y_deg: float,
        default_height_percent: float = 50.0,
    ) -> float:
        model = self.get_model(tool_type)
        return model.inverse_predict(
            target_depth_mm,
            hardness_mpa,
            speed_kph,
            slope_x_deg,
            slope_y_deg,
            default_height_percent,
        )

    def save_models(self, save_dir="."):
        import os

        save_dict = {}
        for t, m in self.models.items():
            save_dict[f"{t}_theta"] = m.get_params()
            save_dict[f"{t}_count"] = m.training_count  # 保存计数器
        np.savez(os.path.join(save_dir, "height_models.npz"), **save_dict)

    def load_models(self, filepath):
        data = np.load(filepath)
        for key in data.files:
            if key.endswith("_theta"):
                tool = key[:-6]
                theta = data[key]
                model = self.get_model(tool)
                model.set_params(theta)
                # 尝试恢复对应的训练次数（不存在则保持0）
                count_key = f"{tool}_count"
                if count_key in data:
                    model.training_count = int(data[count_key])

    def set_global_model(self, tool_type, theta_list):
        model = self.get_model(tool_type)
        model.set_theta(theta_list)
