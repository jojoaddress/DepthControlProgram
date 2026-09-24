#!/usr/bin/env python3
# sim_client.py - 模拟客户端，不依赖任何硬件，用于联邦学习实验

import numpy as np
import time
import requests
import threading
import sys
import os
import json
from datetime import datetime

# 导入你的正向深度模型类
from ForwardDepthModel import MultiToolHeightModel

# ========== 配置（每个客户端可独立修改） ==========
SERVER_URL = "http://127.0.0.1:5000"  # 替换为电脑实际IP
CLIENT_ID = "orangepi_A"  # 区分客户端，如 A 和 B
TOOL_TYPE = "subsoiler"  # 机具类型，保持一致
UPLOAD_INTERVAL = 20  # 每20次本地更新上传一次
DOWNLOAD_INTERVAL = 30  # 每30秒拉取一次全局模型
TOTAL_TRAIN_STEPS = 500  # 总训练步数（模拟500次作业更新）
SLEEP_BETWEEN_STEPS = 0.5  # 步间隔秒

# ========== 区域特性定义（不同客户端使用不同的真实生成函数） ==========
# 真实模型参数: [偏置, 高度系数, 硬度系数, 速度系数, 坡度X系数, 坡度Y系数, 高度×硬度系数]
# 这些系数模拟了该区域的物理特性（土壤、地形、作业习惯等）
# 客户端A（区域1）
REGION_A_THETA = [10.0, 1.2, -4.0, -0.5, -0.2, -0.1, 0.12]
# 客户端B（区域2）
REGION_B_THETA = [5.0, 0.8, -7.0, -0.7, -0.3, -0.2, 0.20]

# 根据 CLIENT_ID 选择对应的真实参数（也可以写在单独配置文件中）
if CLIENT_ID == "orangepi_A":
    TRUE_THETA = REGION_A_THETA
elif CLIENT_ID == "orangepi_B":
    TRUE_THETA = REGION_B_THETA
else:
    # 默认
    TRUE_THETA = REGION_A_THETA

# ========== 特征取值范围（模拟真实作业环境） ==========
# 悬挂高度 % (0-100)
HEIGHT_RANGE = (30, 80)
# 土壤硬度 MPa (随区域有所不同，这里为简单使用统一范围，真实映射已体现差异)
HARDNESS_RANGE = (2.0, 4.0)
# 作业速度 km/h
SPEED_RANGE = (4, 8)
# 坡度 X 度
SLOPE_X_RANGE = (-5, 5)
# 坡度 Y 度
SLOPE_Y_RANGE = (-5, 5)


# ========== 模拟客户端类 ==========
class SimulatedClient:
    def __init__(
        self,
        tool_type,
        server_url,
        client_id,
        upload_interval,
        download_interval,
        true_theta,
    ):
        self.tool_type = tool_type
        self.server_url = server_url
        self.client_id = client_id
        self.upload_interval = upload_interval
        self.download_interval = download_interval
        self.true_theta = np.array(true_theta)
        self.model = MultiToolHeightModel(
            forget_factor=0.98, delta=100.0, ridge_penalty=1e-4
        )
        self.local_update_count = 0
        self.last_upload_count = 0
        self.step_count = 0

        # 下载初始全局模型
        self.download_global_model()
        # 启动后台定时拉取线程
        if download_interval > 0:
            self.keep_running = True
            self.pull_thread = threading.Thread(
                target=self.periodic_download, daemon=True
            )
            self.pull_thread.start()

    def download_global_model(self):
        try:
            resp = requests.get(
                f"{self.server_url}/download/{self.tool_type}", timeout=5
            )
            if resp.status_code == 200:
                data = resp.json()
                global_theta = data["theta"]
                self.model.set_global_model(self.tool_type, global_theta)
                print(f"[{self.client_id}] 成功加载全局模型 theta={global_theta}")
            else:
                print(f"[{self.client_id}] 服务器尚无该机具模型，将从零训练")
        except Exception as e:
            print(f"[{self.client_id}] 下载全局模型失败: {e}")

    def periodic_download(self):
        while self.keep_running:
            time.sleep(self.download_interval)
            try:
                resp = requests.get(
                    f"{self.server_url}/download/{self.tool_type}", timeout=3
                )
                if resp.status_code == 200:
                    global_theta = resp.json()["theta"]
                    self.model.set_global_model(self.tool_type, global_theta)
                    print(f"[{self.client_id}] 定时拉取全局模型成功")
            except Exception as e:
                pass  # 静默失败

    def upload_local_model(self):
        try:
            model = self.model.get_model(self.tool_type)
            theta, cnt = model.get_theta_with_count()
            payload = {
                "tool_type": self.tool_type,
                "theta": theta,
                "training_count": cnt,
                "client_id": self.client_id,
            }
            resp = requests.post(f"{self.server_url}/upload", json=payload, timeout=3)
            if resp.status_code == 200:
                self.last_upload_count = cnt
                print(f"[{self.client_id}] 上传模型成功 (总训练次数={cnt})")
            else:
                print(f"[{self.client_id}] 上传失败，状态码={resp.status_code}")
        except Exception as e:
            print(f"[{self.client_id}] 上传异常: {e}")

    def generate_training_sample(self):
        """随机生成特征，并用真实模型计算真实深度（加少量噪声）"""
        height = np.random.uniform(*HEIGHT_RANGE)
        hardness = np.random.uniform(*HARDNESS_RANGE)
        speed = np.random.uniform(*SPEED_RANGE)
        slope_x = np.random.uniform(*SLOPE_X_RANGE)
        slope_y = np.random.uniform(*SLOPE_Y_RANGE)
        # 构造特征向量 (与 ForwardDepthModel 中一致)
        x = np.array(
            [1.0, height, hardness, speed, slope_x, slope_y, height * hardness]
        )
        true_depth = max(0.0, np.dot(self.true_theta, x) + np.random.normal(0, 2.0))
        return height, hardness, speed, slope_x, slope_y, true_depth

    def train_one_step(self):
        """进行一次本地训练"""
        height, hardness, speed, slope_x, slope_y, true_depth = (
            self.generate_training_sample()
        )
        model = self.model.get_model(self.tool_type)
        pred_before = model.predict(height, hardness, speed, slope_x, slope_y)
        model.update(height, hardness, speed, slope_x, slope_y, true_depth)
        self.local_update_count = model.training_count
        self.step_count += 1
        error = true_depth - pred_before
        print(
            f"[{self.client_id}] Step {self.step_count:3d}: Height={height:5.1f}% TrueDepth={true_depth:5.1f}mm "
            f"Pred={pred_before:5.1f}mm Error={error:+5.1f}mm | 本地训练次数={self.local_update_count}"
        )

        # 检查是否需要上传
        if self.local_update_count - self.last_upload_count >= self.upload_interval:
            self.upload_local_model()

    def evaluate_cross_region(self, target_client_theta, num_samples=100):
        """
        评估当前模型在另一个区域（使用 target_client_theta 生成数据）上的 MAE
        此方法用于离线实验对比，不参与训练。
        """
        model = self.model.get_model(self.tool_type)
        errors = []
        for _ in range(num_samples):
            height = np.random.uniform(*HEIGHT_RANGE)
            hardness = np.random.uniform(*HARDNESS_RANGE)
            speed = np.random.uniform(*SPEED_RANGE)
            slope_x = np.random.uniform(*SLOPE_X_RANGE)
            slope_y = np.random.uniform(*SLOPE_Y_RANGE)
            x = np.array(
                [1.0, height, hardness, speed, slope_x, slope_y, height * hardness]
            )
            true_depth = max(0.0, np.dot(target_client_theta, x))
            pred_depth = model.predict(height, hardness, speed, slope_x, slope_y)
            errors.append(abs(true_depth - pred_depth))
        return np.mean(errors)

    def run(self, total_steps):
        for step in range(total_steps):
            self.train_one_step()
            time.sleep(SLEEP_BETWEEN_STEPS)
        self.keep_running = False  # 停止拉取线程

    def final_evaluation(self, other_client_theta):
        mae = self.evaluate_cross_region(other_client_theta)
        print(f"\n[{self.client_id}] 最终跨区域评估 MAE = {mae:.2f} mm")
        return mae


# ========== 主程序 ==========
if __name__ == "__main__":
    # 解析命令行参数可简单支持不同客户端 ID
    if len(sys.argv) > 1:
        CLIENT_ID = sys.argv[1]
    print(f"启动模拟客户端: {CLIENT_ID}, 真实区域参数: {TRUE_THETA}")

    client = SimulatedClient(
        TOOL_TYPE, SERVER_URL, CLIENT_ID, UPLOAD_INTERVAL, DOWNLOAD_INTERVAL, TRUE_THETA
    )
    try:
        client.run(TOTAL_TRAIN_STEPS)
        # 可选：最终打印跨区域评估（需要传入另一个客户端的真实参数）
        # 这里简化：可以在实验结束后单独用脚本加载两个模型的参数进行评估
    except KeyboardInterrupt:
        print("用户中断")
        client.keep_running = False
