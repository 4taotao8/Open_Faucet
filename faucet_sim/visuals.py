"""Isaac 原生可视化反馈：实际阀角、阶段和符号水柱；不做流体求解。"""
import numpy as np
from pxr import Gf, UsdGeom


def water_state(yaw, lift):
    """由实测弧度角计算理想开度/流量/温度，返回普通字典。"""
    opening = float(np.clip(lift / np.deg2rad(35.), 0., 1.))
    hot_fraction = float(np.clip((yaw + np.pi/4) / (np.pi/2), 0., 1.))
    return {'opening': opening, 'flow_lpm': 6.*opening,
            'temperature_c': 20.+40.*hot_fraction if opening > 1e-4 else None}


class SimulationDisplay:
    """持有面板和几何引用；显示数值始终来自物理反馈而不是目标值。"""

    def __init__(self, stage, config, show_panel):
        """stage 为正在运行的场景；无窗口模式只更新水柱形状。"""
        self.stage, self.window = stage, None
        self.config = config
        self.water = UsdGeom.Cylinder.Define(stage, '/World/Faucet/WaterIndicator')
        self.water.CreateRadiusAttr(.003)
        self.water.CreateHeightAttr(.12)
        self.water.CreateAxisAttr('Z')
        # 水柱是无碰撞的符号显示。位置附属于龙头根，跟随场景摆放。
        self.water.AddTranslateOp().Set(Gf.Vec3d(0, .12, .075))
        self.water.CreateDisplayColorAttr([Gf.Vec3f(.12, .55, .95)])
        self.water.CreateVisibilityAttr('invisible')
        if show_panel:
            import omni.ui as ui
            self.window = ui.Window('Faucet trajectory demo', width=350, height=200)
            with self.window.frame:
                with ui.VStack(spacing=6):
                    ui.Label('Right arm | planned trajectories | ideal grasp')
                    self.phase_label = ui.Label('Preparing')
                    self.angle_label = ui.Label('Yaw: 0 deg | Lift: 0 deg')
                    self.flow_label = ui.Label('Flow: 0 L/min | Closed')
                    self.grasp_label = ui.Label('Grasp: released')

    def update(self, yaw, lift, phase, grasp_active):
        """输入实测阀角和流程状态；更新显示并返回水状态。"""
        state = water_state(yaw, lift)
        self.water.GetVisibilityAttr().Set('inherited' if state['opening'] > .005 else 'invisible')
        self.water.GetRadiusAttr().Set(.001 + .002 * state['opening'])
        if self.window:
            self.phase_label.text = f'Phase: {phase}'
            self.angle_label.text = f'Yaw: {np.rad2deg(yaw):.1f} deg | Lift: {np.rad2deg(lift):.1f} deg'
            temperature = 'Closed' if state['temperature_c'] is None else f"{state['temperature_c']:.1f} C"
            self.flow_label.text = f"Flow: {state['flow_lpm']:.2f} L/min | {temperature}"
            self.grasp_label.text = 'Grasp: attached (ideal constraint)' if grasp_active else 'Grasp: released'
        return state
