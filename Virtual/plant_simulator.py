class PlantSimulator:
    """农具连杆与被控对象仿真器（支持一阶液压延迟响应）"""

    def __init__(self, tau: float = 0.8, angle_gain: float = 0.4):
        self.tau = tau                      # 液压响应时间常数（秒）
        self.angle_gain = angle_gain        # 悬挂百分比转化机具俯仰角增益

        # 物理状态量
        self.current_height = 50.0          # 当前实际悬挂高度 (%)
        self.target_height = 50.0           # 控制器指令悬挂高度 (%)
        self.veh_pitch = 0.0                # 车身纵向俯仰角 β (度)
        self.veh_roll = 0.0                 # 车身横滚角 (度)
        self.impl_pitch = 0.0               # 机具俯仰角 α (度)

        self.closed_loop_enabled = False    # 是否启用物理闭环

    def set_command_height(self, cmd_height: float):
        """接收主程序下发的悬挂目标高度 (%)"""
        self.target_height = max(0.0, min(100.0, float(cmd_height)))

    def step(self, dt: float, manual_alpha: float, manual_beta: float):
        """动力学步进更新"""
        self.veh_pitch = manual_beta

        if self.closed_loop_enabled:
            # 一阶惯性环节模拟液压动作: dh/dt = (target - current) / tau
            dh = (self.target_height - self.current_height) / self.tau * dt
            self.current_height += dh

            # 机构连杆映射：悬挂抬高 -> 农具俯仰角增加（浅耕）；悬挂降低 -> 农具俯仰角减小（深耕）
            # 基准 50% 对应 0° 增量
            kinematic_alpha = -(self.current_height - 50.0) * self.angle_gain
            self.impl_pitch = kinematic_alpha + manual_alpha
        else:
            self.impl_pitch = manual_alpha