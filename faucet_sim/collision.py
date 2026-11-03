"""修复夹爪开口碰撞近似，监测 PhysX 接触距离；不屏蔽夹爪/把手碰撞。"""
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, Gf, UsdShade
import numpy as np
from scipy.spatial import ConvexHull, QhullError
from .geometry import world_pose, inverse

GRIPPER_PATHS = ['/World/Robot/arm_r_link_gripper', '/World/Robot/arm_r_link_finger']
HANDLE_PATH = '/World/Faucet/handle'


def configure_grasp_colliders(stage):
    """把原网格沿长度每 2 mm 裁成薄片凸包，保留手指内侧曲线与开口。

    原 STL 存在开口面；通用体素分解会填充这些开口。这里直接裁剪原三角形，
    再对每个薄片取凸包；不增加依赖，不修改源资产或视觉网格。
    """
    for path in GRIPPER_PATHS:
        stage.GetPrimAtPath(path + '/collisions').SetInstanceable(False)
    for path in GRIPPER_PATHS + [HANDLE_PATH]:
        root = stage.GetPrimAtPath(path)
        # 清除任何旧版过滤关系；默认输出层中并不存在该关系。
        if root.HasAPI(UsdPhysics.FilteredPairsAPI):
            UsdPhysics.FilteredPairsAPI(root).CreateFilteredPairsRel().ClearTargets(True)
        meshes = [p for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Mesh) and
                  ('/collisions/' in str(p.GetPath()) or path == HANDLE_PATH)]
        for prim in list(Usd.PrimRange(root)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        parts = []
        for prim in meshes:
            mesh = UsdGeom.Mesh(prim)
            vertices = np.array(mesh.GetPointsAttr().Get(), dtype=float)
            transform = inverse(world_pose(root)) @ world_pose(prim)
            vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            indices = np.array(mesh.GetFaceVertexIndicesAttr().Get())
            counts = np.array(mesh.GetFaceVertexCountsAttr().Get())
            triangles, offset = [], 0
            for count in counts:
                face = indices[offset:offset+count]
                triangles.extend([[face[0], face[k], face[k+1]] for k in range(1, count-1)])
                offset += count
            parts.extend(sliced_hulls(vertices[np.asarray(triangles)], 1 if path == HANDLE_PATH else 2))
        folder = UsdGeom.Xform.Define(stage, path + '/contact_hulls')
        folder.CreateVisibilityAttr('invisible')
        for index, (vertices, faces) in enumerate(parts):
            mesh = UsdGeom.Mesh.Define(stage, str(folder.GetPath()) + f'/piece_{index:03d}')
            mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in vertices])
            mesh.CreateFaceVertexCountsAttr([3] * len(faces))
            mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist())
            prim = mesh.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
            prim.AddAppliedSchema('PhysxConvexHullCollisionAPI')
            prim.CreateAttribute('physxConvexHullCollision:hullVertexLimit', Sdf.ValueTypeNames.Int).Set(128)
            prim.AddAppliedSchema('PhysxCollisionAPI')
            prim.CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.0004)
            prim.CreateAttribute('physxCollision:restOffset', Sdf.ValueTypeNames.Float).Set(.00005)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(
                stage.GetPrimAtPath('/World/ContactMaterial')), materialPurpose='physics')


def sliced_hulls(triangles, axis, thickness=.002):
    """沿指定轴裁剪三角形网格；输出每段凸包顶点和三角面，不跨段填平凹面。"""
    minimum, maximum = triangles[:, :, axis].min(), triangles[:, :, axis].max()
    bounds = np.linspace(minimum, maximum, int(np.ceil((maximum-minimum)/thickness)) + 1)
    parts = []
    for low, high in zip(bounds[:-1], bounds[1:]):
        selected = triangles[(triangles[:, :, axis].min(1) <= high) &
                             (triangles[:, :, axis].max(1) >= low)]
        points = selected.reshape(-1, 3)
        points = points[(points[:, axis] >= low) & (points[:, axis] <= high)]
        buffers = [points]
        for edge in [(0, 1), (1, 2), (2, 0)]:
            a, b = selected[:, edge[0]], selected[:, edge[1]]
            for plane in [low, high]:
                crossing = (a[:, axis]-plane) * (b[:, axis]-plane) < 0
                start, end = a[crossing], b[crossing]
                fraction = (plane-start[:, axis]) / (end[:, axis]-start[:, axis])
                buffers.append(start + fraction[:, None] * (end-start))
        points = np.unique(np.round(np.vstack(buffers), 8), axis=0)
        if len(points) < 4:
            continue
        try:
            hull = ConvexHull(points)
        except QhullError:
            continue
        mapping = np.zeros(len(points), dtype=int)
        mapping[hull.vertices] = np.arange(len(hull.vertices))
        parts.append((points[hull.vertices], mapping[hull.simplices]))
    return parts


class ContactMonitor:
    """逐步读取两个夹爪刚体相对龙头的接触距离、冲量和最大穿透深度。"""

    def __init__(self, world):
        """在 world.reset 前订阅 PhysX 原生接触报告，按刚体路径识别碰撞双方。"""
        from pxr import PhysxSchema
        from omni.physx import get_physx_simulation_interface
        stage = world.stage
        for path in GRIPPER_PATHS:
            PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)
        self.physics_dt = float(world.get_physics_dt())
        self.subscription = get_physx_simulation_interface().subscribe_full_contact_report_events(self.on_contact)
        self.max_penetration = 0.
        self.handle_contact_steps = 0
        self.max_normal_force = 0.
        self.max_friction_force = 0.
        self.total_friction_impulse = 0.
        self.pairs = set()
        self.begin_step()

    def begin_step(self):
        """清零本物理步计数，保留全程最大穿透统计。"""
        self.depth, self.handle_count, self.impulse = 0., 0, 0.
        self.fixed_count, self.moving_count = 0, 0
        self.callback_batches = 0
        self.fixed_impulse, self.moving_impulse, self.friction_impulse = 0., 0., 0.
        self.last = {'contact_points': 0, 'contact_impulse_ns': 0., 'penetration_mm': 0.,
                     'fixed_contact_points': 0, 'moving_contact_points': 0,
                     'fixed_normal_force_n': 0., 'moving_normal_force_n': 0., 'friction_force_n': 0.}

    def on_contact(self, headers, contacts, friction_anchors):
        """由 PhysX 回调提供接触点；同时覆盖把手、龙头主体和台面等对象。"""
        from pxr import PhysicsSchemaTools
        self.callback_batches += 1
        for header in headers:
            actors = [str(PhysicsSchemaTools.intToSdfPath(actor)) for actor in [header.actor0, header.actor1]]
            if not any(actor in GRIPPER_PATHS for actor in actors):
                continue
            pair = tuple(sorted(actors))
            if header.num_contact_data and pair not in self.pairs:
                print('[接触] ' + ' <-> '.join(pair), flush=True)
                self.pairs.add(pair)
            start, count = header.contact_data_offset, header.num_contact_data
            for contact in contacts[start:start+count]:
                self.depth = max(self.depth, max(0., -float(contact.separation)))
                if HANDLE_PATH in actors:
                    self.handle_count += 1
                    if GRIPPER_PATHS[0] in actors:
                        self.fixed_count += 1
                        self.fixed_impulse += float(np.linalg.norm(contact.impulse))
                    if GRIPPER_PATHS[1] in actors:
                        self.moving_count += 1
                        self.moving_impulse += float(np.linalg.norm(contact.impulse))
                    self.impulse += float(np.linalg.norm(contact.impulse))
            if HANDLE_PATH in actors:
                first, count = header.friction_anchors_offset, header.num_friction_anchors_data
                for anchor in friction_anchors[first:first+count]:
                    self.friction_impulse += float(np.linalg.norm(anchor.impulse))

    def sample(self):
        """返回接触点数量、冲量和穿透深度；穿透大于 0.8 mm 则失败。"""
        self.max_penetration = max(self.max_penetration, self.depth)
        if self.handle_count:
            self.handle_contact_steps += 1
        elapsed = self.physics_dt * max(1, self.callback_batches)
        fixed_force, moving_force = self.fixed_impulse/elapsed, self.moving_impulse/elapsed
        friction_force = self.friction_impulse/elapsed
        self.max_normal_force = max(self.max_normal_force, fixed_force, moving_force)
        self.max_friction_force = max(self.max_friction_force, friction_force)
        self.total_friction_impulse += self.friction_impulse
        self.last = {'contact_points': self.handle_count, 'contact_impulse_ns': self.impulse,
                     'penetration_mm': self.depth * 1000., 'fixed_contact_points': self.fixed_count,
                     'moving_contact_points': self.moving_count, 'fixed_normal_force_n': fixed_force,
                     'moving_normal_force_n': moving_force, 'friction_force_n': friction_force}
        if self.depth > .0008:
            raise RuntimeError(f'夹爪碰撞穿透超过容限：{self.depth*1000:.3f} mm')
        return self.last
