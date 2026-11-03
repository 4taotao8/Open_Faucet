# 单把双自由度混水龙头 / Isaac Sim USD

按参考照片的操作结构原创建模：顶部只有一个把手，左右转动调整冷热，上下抬动调整开关和流量。外形为银色圆柱龙头、前伸扁平出水嘴和顶部扁平把手；不含照片中的商标。几何、材质和物理配置均为本次新建，未复用之前下载的水龙头。

## 文件

| 文件 | 用途 |
|---|---|
| `single_lever_faucet.usd` | 独立资产，内嵌全部网格和材质，可单独复制使用 |
| `single_lever_faucet.usda` | 同一资产的可读文本版本 |
| `demo_scene.usda` | 演示场景，含龙头、台面、重力、灯光和相机 |
| `control.py` | 两个关节的目标控制、被动模式、状态计算及滑块面板 |
| `demo_isaacsim.py` | 在 Isaac Sim Python 环境启动演示 |
| `preview_closed.png`、`preview_open.png` | Blender 渲染的关闭 / 抬起并旋转把手预览 |
| `validation.json` | 文件、物理结构及控制逻辑检查结果 |

## 打开使用

在 Isaac Sim 中用 **File → Open** 打开 `demo_scene.usda`，点击 **Play**。可在视口相机列表选择 `/World/Camera`。独立使用时打开 `single_lever_faucet.usd`，或把它引用到已有场景。

默认保持关闭、冷热居中。在属性面板中修改以下关节的 **Angular Drive → Target Position**，把手将由物理驱动转至目标角度：

| 关节 | 运动 | 范围 | 含义 |
|---|---|---|---|
| `temperature_yaw` | 绕竖直 Z 轴左右旋转 | −45°～+45° | −45° 全冷、0° 混合、+45° 全热 |
| `flow_lift` | 绕随调温部件转动的 X 轴抬起 | 0°～35° | 0° 关闭、35° 最大流量 |

独立资产的关节路径为 `/Faucet/joints/...`；演示场景路径为 `/World/Faucet/joints/...`。

两个关节是串联关系：固定主体 → 调温部件 → 同一个把手。因此左右旋转以后，上下抬动的方向也随把手转动，不会出现两个独立把手的操作。

## 用滑块控制

在 Isaac Sim 的 Script Editor 执行以下内容，保留 `faucet_panel` 变量；点击 Play 后拖动滑块：

```python
import sys
sys.path.insert(0, r'D:\Users\78165\Documents\Codex\single_lever_faucet')
from control import show_panel
faucet_panel = show_panel('/World/Faucet')
# 打开独立 USD 时使用 show_panel('/Faucet')。
```

也可直接指定开度与水温：

```python
import omni.usd
from control import set_handle
stage = omni.usd.get_context().get_stage()
set_handle(stage, opening=0.6, temperature_c=40, root_path='/World/Faucet')
set_handle(stage, opening=0.0, temperature_c=40, root_path='/World/Faucet')  # 关闭
```

如果使用 Isaac Sim 4.5 / 5.x 自带的 Python，可在其安装目录运行：

```powershell
.\python.bat "D:\Users\78165\Documents\Codex\single_lever_faucet\demo_isaacsim.py"
```

演示脚本按 Isaac Sim 4.5 / 5.x 接口编写。本机未启动 Isaac Sim，其他版本的界面或启动入口可能需要调整；核心 USD 采用标准的 UsdPhysics 旋转关节和刚体结构。

## 机械手接触与强化学习

主体、调温部件和把手共 **3 个刚体**，由 **2 个旋转关节** 串联；主体用固定关节连接世界。把手有凸包碰撞体，主体由多个凸包组成。自身碰撞关闭，仍可与机械手及场景物体碰撞。质量和惯量均为合理估算值，供初始仿真使用。

默认启用位置保持，适合脚本控制。让机器人直接拨动把手时，先关闭位置保持，避免驱动与机械手对抗：

```python
from control import set_passive
set_passive(stage, enabled=True, root_path='/World/Faucet')
# 恢复位置保持：enabled=False。
```

使用 Isaac Lab 时，按关节名解析索引，不要假定关节数组顺序；两个受控关节名是 `temperature_yaw`、`flow_lift`。采用隐式驱动时可以从 USD 读取默认驱动参数，再按机器人实际力矩调校。

读取实际关节位置后计算当前开关和水温：

```python
from control import state_from_positions
state = state_from_positions(temperature_yaw_rad, flow_lift_rad)
# 返回 opening、flow_lpm、flowing、temperature_c、hot_fraction。
```

传入值必须为物理步之后读取的实际关节角度，单位为 **弧度**。USD 属性面板的角度和驱动目标则使用 **度**。`set_handle()` 返回的是目标状态；`faucet:target*` 属性也只记录目标，不代表把手已到位。

## 尺寸与出水逻辑

- 米制、Z 向上，+Y 为出水嘴朝向。主体底部位于 Z=0；关闭时整体约高 0.22 m、宽 0.062 m、前伸约 0.135 m。属于设计尺寸，不是图片产品的实测 CAD。
- 默认冷水 20℃、热水 60℃、最大总流量 6 L/min，可修改资产根节点上的 `faucet:coldSupplyC`、`hotSupplyC`、`maxFlowLpm`。
- 总流量与抬升角度线性对应；冷热比例与左右转角线性对应。全关时温度返回 `None`。
- 不含流体粒子、喷射水柱、热传导或真实阀芯流量曲线。需要的水量和温度反馈由任务逻辑计算。

## 已完成的检查

使用 OpenUSD 25.5 的物理解析器识别出 1 个 articulation、3 个刚体、1 个固定关节、2 个旋转关节和 5 个网格碰撞体，描述符全部有效。另检查了关节连接、初始关节坐标对齐、惯量、角度限制、冷热映射、被动模式和把手运动方向。独立 USD 无外部文件依赖；关闭和抬起姿态已渲染并目视检查。

**尚未在 Isaac Sim / PhysX 中运行验证**。接触稳定性、驱动增益和真实机器人拨动力矩需要在你的 Isaac Sim 场景中调试。无需把 OpenUSD 生成工具的依赖复制到 Isaac Sim，Isaac Sim 自带 `pxr`。

原先下载的 `faucet_usd_models` 目录和下载缓存已移入 Windows 回收站；当前交付目录只有此次原创资产及其生成、控制和检查文件。
