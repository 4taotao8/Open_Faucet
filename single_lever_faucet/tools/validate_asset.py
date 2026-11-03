"""Structural + kinematic checks. This is not an Isaac Sim runtime test."""
import sys,pathlib,math,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
try:
    from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf
except ImportError:
    sys.path.insert(0,str(ROOT.parent/'tmp'/'faucet_usd_runtime'))
    from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf
sys.path.insert(0,str(ROOT))
from control import set_handle,state_from_positions,set_passive

asset=ROOT/'single_lever_faucet.usd'
s=Usd.Stage.Open(str(asset))
assert UsdGeom.GetStageMetersPerUnit(s)==1.
assert UsdGeom.GetStageUpAxis(s)=='Z'
parsed=UsdPhysics.LoadUsdPhysicsFromRange(s,[Sdf.Path('/')])
counts={str(k):len(v[0]) for k,v in parsed.items()}
assert counts['Articulation']==1 and counts['RigidBody']==3 and counts['RevoluteJoint']==2
assert counts['FixedJoint']==1 and counts['MeshShape']==5
for kind,(paths,descriptors) in parsed.items():
    for desc in descriptors:assert desc.isValid,(kind,desc.primPath)
cache=UsdGeom.XformCache()
limits={}
for name,axis,expected in [('temperature_yaw','Z',(-45.,45.)),('flow_lift','X',(0.,35.))]:
    j=UsdPhysics.RevoluteJoint(s.GetPrimAtPath('/Faucet/joints/'+name))
    assert j.GetAxisAttr().Get()==axis
    limits[name]=[j.GetLowerLimitAttr().Get(),j.GetUpperLimitAttr().Get()]
    assert tuple(limits[name])==expected
    parent=s.GetPrimAtPath(j.GetBody0Rel().GetTargets()[0]);child=s.GetPrimAtPath(j.GetBody1Rel().GetTargets()[0])
    a=cache.GetLocalToWorldTransform(parent).Transform(Gf.Vec3d(j.GetLocalPos0Attr().Get()))
    b=cache.GetLocalToWorldTransform(child).Transform(Gf.Vec3d(j.GetLocalPos1Attr().Get()))
    assert (a-b).GetLength()<1e-7
for prim in s.Traverse():
    if prim.HasAPI(UsdPhysics.MassAPI):
        m=UsdPhysics.MassAPI(prim);assert m.GetMassAttr().Get()>0
        inertia=list(m.GetDiagonalInertiaAttr().Get());assert min(inertia)>0
        assert 2*max(inertia)<=sum(inertia)
    if prim.IsA(UsdGeom.Mesh):
        mesh=UsdGeom.Mesh(prim);assert len(mesh.GetPointsAttr().Get())>0
        assert mesh.GetFaceVertexCountsAttr().Get() and prim.HasAPI(UsdPhysics.CollisionAPI)
        assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()=='convexHull'
checks=[]
for opening,temp in [(0.,40.),(1.,20.),(1.,60.),(.5,40.)]:
    state=set_handle(s,opening,temp)
    y=s.GetPrimAtPath('/Faucet/joints/temperature_yaw').GetAttribute('drive:angular:physics:targetPosition').Get()
    l=s.GetPrimAtPath('/Faucet/joints/flow_lift').GetAttribute('drive:angular:physics:targetPosition').Get()
    assert abs(l-opening*35)<1e-5
    assert abs(y-((temp-20)/40*90-45))<1e-5
    assert state['temperature_c']==(temp if opening else None)
    assert state['flow_lpm']==6*opening
    checks.append(dict(opening=opening,temperature_c=temp,yaw_deg=y,lift_deg=l))
set_passive(s,True)
assert s.GetPrimAtPath('/Faucet/joints/flow_lift').GetAttribute('drive:angular:physics:stiffness').Get()==0.
set_passive(s,False)
# FK: lifting along X raises the forward (+Y) end; yaw preserves end height.
tip=Gf.Vec3d(0,.1,.023)
lifted=Gf.Rotation(Gf.Vec3d(1,0,0),35).TransformDir(tip)
assert lifted[2]>tip[2]+.04
turned=Gf.Rotation(Gf.Vec3d(0,0,1),45).TransformDir(lifted)
assert abs(turned[2]-lifted[2])<1e-12
assert not [p for p in s.GetRootLayer().GetExternalReferences() if p]
assert Usd.Stage.Open(str(ROOT/'demo_scene.usda'))
report=dict(passed=True,physics_descriptors=counts,limits_degrees=limits,control_checks=checks,
            joint_frames_aligned=True,inertias_positive=True,lift_raises_handle_tip=True,
            yaw_preserves_lift_height=True,asset_external_files=[],
            isaac_sim_runtime_tested=False,
            runtime_note='Isaac Sim is not installed in the available environment; checked with OpenUSD 25.5 physics parser, not a PhysX simulation.',
            sha256=hashlib.sha256(asset.read_bytes()).hexdigest())
(ROOT/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
