"""理想刚性抓取：通过 PhysX 固定约束传力，不给把手写运动动画。"""
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, UsdPhysics
from .geometry import inverse, world_pose, pose_error


def set_joint_frame(joint, side, transform):
    """将局部抓取坐标系写入固定约束的指定端，side 为 0 或 1。"""
    q = Rotation.from_matrix(transform[:3, :3]).as_quat()
    getattr(joint, f'CreateLocalPos{side}Attr')(Gf.Vec3f(*transform[:3, 3]))
    getattr(joint, f'CreateLocalRot{side}Attr')(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))


class IdealGrasp:
    """闭爪且 TCP 到达抓取位姿后建约束；松爪前删除约束。

    这是任务级理想无滑移抓取，不能据此认定摩擦夹持或夹爪力已经验证。
    两个固定基 articulation 通过排除在树外的约束形成闭环。
    """

    def __init__(self, stage, robot_model, faucet_model):
        self.stage, self.robot, self.faucet = stage, robot_model, faucet_model
        self.path = '/World/GraspConstraint'
        self.active = False

    def attach(self, actual_q, actual_valve):
        """输入物理反馈的角度，检查捕获窗口并建立约束；越界直接失败。"""
        tcp = self.robot.forward(actual_q)
        desired = self.faucet.target(*actual_valve)
        distance, angle = pose_error(tcp, desired)
        if distance > .012 or angle > np.deg2rad(8):
            raise RuntimeError(f'抓取失败：TCP 距把手 {distance*1000:.2f} mm，姿态差 {np.rad2deg(angle):.2f}°')
        gripper = world_pose(self.stage.GetPrimAtPath('/World/Robot/arm_r_link_gripper'))
        handle = world_pose(self.stage.GetPrimAtPath('/World/Faucet/handle'))
        joint = UsdPhysics.FixedJoint.Define(self.stage, self.path)
        joint.CreateBody0Rel().SetTargets(['/World/Robot/arm_r_link_gripper'])
        joint.CreateBody1Rel().SetTargets(['/World/Faucet/handle'])
        set_joint_frame(joint, 0, inverse(gripper) @ tcp)
        set_joint_frame(joint, 1, inverse(handle) @ tcp)
        joint.CreateExcludeFromArticulationAttr(True)
        # 固定抓取约束也必须允许两端接触，不能隐式关闭夹爪与把手碰撞。
        joint.CreateCollisionEnabledAttr(True)
        joint.CreateBreakForceAttr(1000.)
        joint.CreateBreakTorqueAttr(100.)
        self.active = True
        return {'capture_position_error_mm': distance*1000, 'capture_rotation_error_deg': float(np.rad2deg(angle))}

    def release(self):
        """解除抓取，让右臂可以独立撤离。"""
        if self.active:
            self.stage.RemovePrim(self.path)
            self.active = False
