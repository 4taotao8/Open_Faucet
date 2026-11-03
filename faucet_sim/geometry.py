"""位姿工具。内部统一使用米、弧度和列向量齐次变换。"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def pose(position=(0, 0, 0), rotation=None):
    """输入位置和 3×3 旋转矩阵，返回 4×4 齐次变换。"""
    result = np.eye(4)
    result[:3, 3] = position
    if rotation is not None:
        result[:3, :3] = rotation
    return result


def inverse(transform):
    """刚体变换求逆；输入和输出均为 4×4 矩阵。"""
    result = pose(rotation=transform[:3, :3].T)
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def gf_pose(position, quaternion):
    """将 USD 的 Gf 位置及 wxyz 四元数转换成列向量变换。"""
    xyz = list(quaternion.GetImaginary())
    return pose(position, Rotation.from_quat(xyz + [quaternion.GetReal()]).as_matrix())


def world_pose(prim):
    """读取 USD 刚体当前世界位姿；USD 行向量矩阵需转置。"""
    from pxr import Usd, UsdGeom
    return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T


def interpolate(a, b, fraction):
    """线性插值位置、球面插值旋转；fraction 位于 [0, 1]。"""
    rotations = Rotation.from_matrix(np.stack([a[:3, :3], b[:3, :3]]))
    rotation = Slerp([0, 1], rotations)(float(fraction)).as_matrix()
    return pose((1 - fraction) * a[:3, 3] + fraction * b[:3, 3], rotation)


def smoothstep(value):
    """五次时间标定：端点速度和加速度均为零。"""
    return value**3 * (10 + value * (-15 + 6 * value))


def pose_error(actual, desired):
    """返回位置误差（米）和旋转误差（弧度）的模。"""
    return (float(np.linalg.norm(actual[:3, 3] - desired[:3, 3])),
            float(np.linalg.norm(Rotation.from_matrix(desired[:3, :3] @ actual[:3, :3].T).as_rotvec())))
