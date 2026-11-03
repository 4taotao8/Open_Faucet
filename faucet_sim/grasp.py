"""纯接触/摩擦抓取状态与打滑测量，不创建机器人到龙头的物理约束。"""
import numpy as np
from pxr import UsdPhysics
from .geometry import inverse, pose_error


class FrictionGrasp:
    """仅登记双侧接触形成的抓取，active 不影响刚体动力学。

    不创建固定关节、D6、吸附，不写入把手位姿；打滑只测量不纠正。
    """

    def __init__(self, stage, robot_model, faucet_model):
        """输入场景和运动学对象；抓取参考由实测关节角建立。"""
        self.stage, self.robot, self.faucet = stage, robot_model, faucet_model
        self.active = False
        self.reference = None
        self.max_slip_mm = 0.
        self.max_slip_deg = 0.

    def assert_no_attachment(self):
        """检查场景不存在连接机器人和水龙头的关节。"""
        connections = []
        for prim in self.stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            endpoints = [str(path) for relation in [joint.GetBody0Rel(), joint.GetBody1Rel()]
                         for path in relation.GetTargets()]
            if any(p.startswith('/World/Robot/') for p in endpoints) and any(
                    p.startswith('/World/Faucet/') for p in endpoints):
                connections.append(str(prim.GetPath()))
        if connections:
            raise RuntimeError(f'纯摩擦模式发现意外的跨对象约束：{connections}')
        return connections

    def begin(self, actual_q, actual_valve):
        """登记已由调用方验证的双侧接触；只保存参考变换，不修改 USD。"""
        self.assert_no_attachment()
        tcp = self.robot.forward(actual_q)
        desired = self.faucet.target(*actual_valve)
        distance, angle = pose_error(tcp, desired)
        if distance > .012 or angle > np.deg2rad(8):
            raise RuntimeError(f'抓取失败：TCP 距把手 {distance*1000:.2f} mm，姿态差 {np.rad2deg(angle):.2f}°')
        self.reference = inverse(tcp) @ self.faucet.handle(*actual_valve)
        self.active = True
        return {'capture_position_error_mm': distance*1000, 'capture_rotation_error_deg': float(np.rad2deg(angle)),
                'attachment_joint_created': False}

    def observe(self, actual_q, actual_valve):
        """返回从抓取开始累计的相对滑移（mm、度）；绝不纠正把手运动。"""
        if not self.active:
            return {'slip_mm': 0., 'slip_deg': 0.}
        current = inverse(self.robot.forward(actual_q)) @ self.faucet.handle(*actual_valve)
        distance, angle = pose_error(current, self.reference)
        self.max_slip_mm = max(self.max_slip_mm, distance*1000.)
        self.max_slip_deg = max(self.max_slip_deg, float(np.rad2deg(angle)))
        return {'slip_mm': distance*1000., 'slip_deg': float(np.rad2deg(angle))}

    def release(self):
        """清除逻辑夹持状态；真正释放由活动手指张开形成。"""
        self.active = False
