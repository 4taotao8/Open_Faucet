"""配置读取与边界检查；不依赖额外的配置框架。"""
import copy
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load_config(path=None):
    """读取 JSON 配置并检查资产、角度、时间和数值；返回独立字典。"""
    path = Path(path) if path else ROOT / 'configs/default.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    # 老配置也以纯摩擦模式运行；不提供自动退回固定约束的路径。
    for key, value in {'grasp_mode': 'friction', 'contact_static_friction': 1.,
                       'contact_dynamic_friction': .8, 'finger_stiffness': 4.,
                       'finger_damping': 1., 'finger_max_effort': .3,
                       'faucet_joint_damping': .001}.items():
        config.setdefault(key, value)
    for key in ('robot_usd', 'faucet_usd'):
        asset = ROOT / config[key]
        if not asset.is_file():
            raise FileNotFoundError(f'资产不存在：{asset}')
        config[key] = str(asset.resolve())
    validate(config)
    return config


def validate(config):
    """拒绝非法任务参数，避免静默截断角度或生成不可执行轨迹。"""
    if config.get('grasp_mode') != 'friction':
        raise ValueError('当前版本仅允许无固定抓取约束的 friction 模式')
    for name in ('contact_static_friction', 'contact_dynamic_friction', 'finger_stiffness',
                 'finger_damping', 'faucet_joint_damping'):
        if not np.isfinite(config[name]) or config[name] < 0:
            raise ValueError(f'{name} 必须为非负有限值')
    if not np.isfinite(config['finger_max_effort']) or config['finger_max_effort'] <= 0:
        raise ValueError('finger_max_effort 必须为正有限值')
    for name, size in [('robot_position', 3), ('faucet_position', 3),
                       ('home_right_deg', 7), ('tcp_in_gripper', 3), ('grasp_in_handle', 3),
                       ('gripper_rpy_in_handle_deg', 3)]:
        values = np.asarray(config[name], dtype=float)
        if values.shape != (size,) or not np.isfinite(values).all():
            raise ValueError(f'{name} 必须是 {size} 个有限数值')
    for name in ('physics_dt', 'knot_dt', 'approach_duration', 'grasp_duration',
                 'transit_duration', 'retreat_duration', 'max_joint_velocity', 'max_joint_acceleration',
                 'pregrasp_distance'):
        if not np.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f'{name} 必须为正有限数值')
    for name in ('faucet_yaw_deg', 'robot_yaw_deg'):
        if not np.isfinite(config[name]):
            raise ValueError(f'{name} 必须是有限数值')
    for name in ('finger_open_deg', 'finger_closed_deg'):
        if not np.isfinite(config[name]) or not 0 <= config[name] <= 90:
            raise ValueError(f'{name} 必须在 [0,90] 度内')
    if not config['operations']:
        raise ValueError('operations 不能为空')
    for operation in config['operations']:
        for name, low, high in [('yaw_deg', -45, 45), ('lift_deg', 0, 35)]:
            value = operation[name]
            if not np.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{name}={value} 超出 [{low}, {high}]')
        if not np.isfinite(operation['duration']) or operation['duration'] <= 0:
            raise ValueError('操作 duration 必须为正有限数值')
    return copy.deepcopy(config)
