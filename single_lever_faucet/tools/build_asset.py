"""Create an original single-lever mixer using native USD geometry and physics."""
import math, json, sys, pathlib
ROOT = pathlib.Path(__file__).resolve().parents[1]
try:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, Sdf, Gf
except ImportError:
    sys.path.insert(0, str(ROOT.parent/'tmp'/'faucet_usd_runtime'))
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, Sdf, Gf

PIVOT = (0., 0., .192)
GEOMETRY = []

def material(stage, path, color, metallic=0., roughness=.3):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat

def mesh(stage, path, points, faces, mat, collision=True, link='body'):
    obj = UsdGeom.Mesh.Define(stage, path)
    obj.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    obj.CreateFaceVertexCountsAttr([len(f) for f in faces])
    obj.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    obj.CreateSubdivisionSchemeAttr('none')
    obj.CreateDoubleSidedAttr(False)
    # Smooth normals on swept surfaces; end caps are discretely authored faces.
    normals = [[0.,0.,0.] for _ in points]
    for face in faces:
        a,b,c = [Gf.Vec3d(*points[i]) for i in face[:3]]
        n = Gf.Cross(b-a,c-a)
        for i in face:
            for k in range(3): normals[i][k] += n[k]
    normalized=[]
    for n in normals:
        v=Gf.Vec3d(*n);v.Normalize();normalized.append(Gf.Vec3f(v))
    obj.CreateNormalsAttr(normalized); obj.SetNormalsInterpolation('vertex')
    obj.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(obj.GetPointsAttr().Get()))
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr('convexHull')
    GEOMETRY.append(dict(path=path,points=points,faces=faces,link=link,
                         color=list(mat.GetPrim().GetCustomData().get('color', [.6,.65,.7]))))
    return obj

def lathe(stage,path,rings,mat,link='body',segments=96):
    pts=[]
    for z,r in rings:
        for i in range(segments):
            a=2*math.pi*i/segments;pts.append((r*math.cos(a),r*math.sin(a),z))
    faces=[]
    for k in range(len(rings)-1):
        for i in range(segments):
            j=(i+1)%segments;faces.append((k*segments+i,k*segments+j,(k+1)*segments+j,(k+1)*segments+i))
    faces.extend([tuple(reversed(range(segments))),tuple((len(rings)-1)*segments+i for i in range(segments))])
    return mesh(stage,path,pts,faces,mat,link=link)

def sweep(stage,path,rings,mat,link='body',segments=64):
    # Rounded rectangle section in XZ, swept along Y. Each ring=(y,z,halfwidth,halfheight).
    pts=[]
    for y,z,w,h in rings:
        for i in range(segments):
            a=2*math.pi*i/segments;c,s=math.cos(a),math.sin(a)
            pts.append((w*math.copysign(abs(c)**.6,c),y,z+h*math.copysign(abs(s)**.6,s)))
    faces=[]
    for k in range(len(rings)-1):
        for i in range(segments):
            j=(i+1)%segments;faces.append((k*segments+i,(k+1)*segments+i,(k+1)*segments+j,k*segments+j))
    faces.extend([tuple(range(segments)),tuple(reversed([(len(rings)-1)*segments+i for i in range(segments)]))])
    return mesh(stage,path,pts,faces,mat,link=link)

def body(stage,path,mass,com,inertia):
    p=UsdGeom.Xform.Define(stage,path).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    m=UsdPhysics.MassAPI.Apply(p);m.CreateMassAttr(mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(*com));m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    m.CreatePrincipalAxesAttr(Gf.Quatf(1.))
    return p

def hinge(stage,name,parent,child,axis,lo,hi,pos0,pos1=(0.,0.,0.)):
    j=UsdPhysics.RevoluteJoint.Define(stage,'/Faucet/joints/'+name)
    j.CreateBody0Rel().SetTargets([parent]);j.CreateBody1Rel().SetTargets([child])
    j.CreateAxisAttr(axis);j.CreateLocalPos0Attr(Gf.Vec3f(*pos0));j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    j.CreateLocalRot0Attr(Gf.Quatf(1.));j.CreateLocalRot1Attr(Gf.Quatf(1.))
    j.CreateLowerLimitAttr(lo);j.CreateUpperLimitAttr(hi);j.CreateCollisionEnabledAttr(False)
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular')
    d.CreateTypeAttr('force');d.CreateStiffnessAttr(.06);d.CreateDampingAttr(.005)
    d.CreateMaxForceAttr(1.5);d.CreateTargetPositionAttr(0.)
    j.GetPrim().SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(['PhysicsDriveAPI:angular','PhysxJointAPI']))
    j.GetPrim().CreateAttribute('physxJoint:jointFriction',Sdf.ValueTypeNames.Float).Set(.015)
    j.GetPrim().CreateAttribute('physxJoint:maxJointVelocity',Sdf.ValueTypeNames.Float).Set(180.)
    return j

def build():
    ROOT.mkdir(exist_ok=True)
    path=ROOT/'single_lever_faucet.usda'
    stage=Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    stage.SetMetadata('kilogramsPerUnit',1.)
    root=UsdGeom.Xform.Define(stage,'/Faucet').GetPrim();stage.SetDefaultPrim(root)
    root.SetDocumentation('Original single-lever mixer. +Y is forward. yaw Z controls temperature; lift X controls flow. No liquid or heat solver.')
    # Ancestor root API allows predictable /Faucet articulation discovery.
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(['PhysicsArticulationRootAPI','PhysxArticulationAPI']))
    for n,t,v in [('enabledSelfCollisions',Sdf.ValueTypeNames.Bool,False),('solverPositionIterationCount',Sdf.ValueTypeNames.Int,32),('solverVelocityIterationCount',Sdf.ValueTypeNames.Int,8)]:
        root.CreateAttribute('physxArticulation:'+n,t).Set(v)
    for n,v in [('targetOpening',0.),('targetTemperatureC',40.),('coldSupplyC',20.),('hotSupplyC',60.),('maxFlowLpm',6.)]:
        root.CreateAttribute('faucet:'+n,Sdf.ValueTypeNames.Double,custom=True).Set(v)
    chrome=material(stage,'/Faucet/Looks/Chrome',(.65,.68,.72),1.,.16)
    black=material(stage,'/Faucet/Looks/Aerator',(.022,.025,.03),.1,.38)
    red=material(stage,'/Faucet/Looks/Hot',(.7,.02,.015),.05,.25)
    blue=material(stage,'/Faucet/Looks/Cold',(.015,.12,.65),.05,.25)
    fixed=body(stage,'/Faucet/body',1.2,(0.,.018,.083),(.004,.004,.001))
    carrier=body(stage,'/Faucet/temperature_carrier',.06,(0.,0.,-.004),(.000009,.000009,.000012))
    handle=body(stage,'/Faucet/handle',.09,(0.,.042,.014),(.00009,.000018,.0001))
    UsdGeom.Xformable(carrier).AddTranslateOp().Set(Gf.Vec3d(*PIVOT))
    UsdGeom.Xformable(handle).AddTranslateOp().Set(Gf.Vec3d(*PIVOT))
    lathe(stage,'/Faucet/body/MountingFlange',[(0.,.028),(.001,.030),(.004,.031),(.006,.030),(.007,.027)],chrome)
    lathe(stage,'/Faucet/body/BodyShell',[(.005,.026),(.009,.027),(.025,.0265),(.07,.025),(.115,.024),(.145,.025),(.163,.025),(.176,.023),(.182,.021),(.183,.020)],chrome)
    sweep(stage,'/Faucet/body/Spout',[(.006,.139,.021,.014),(.024,.139,.025,.014),(.050,.140,.026,.012),(.085,.140,.026,.011),(.123,.137,.025,.010),(.133,.136,.024,.009),(.135,.136,.022,.007)],chrome)
    # Dark downward outlet recessed under the spout end (decorative, noncolliding).
    outlet=UsdGeom.Cylinder.Define(stage,'/Faucet/body/Aerator');outlet.CreateRadiusAttr(.012)
    outlet.CreateHeightAttr(.002);outlet.CreateAxisAttr('Z')
    UsdGeom.Xformable(outlet).AddTranslateOp().Set(Gf.Vec3d(0.,.116,.126))
    UsdShade.MaterialBindingAPI.Apply(outlet.GetPrim()).Bind(black)
    lathe(stage,'/Faucet/temperature_carrier/CartridgeCover',[(-.011,.0195),(-.008,.020),(-.003,.020),(0.,.018),(.003,.015)],chrome,'temperature_carrier')
    sweep(stage,'/Faucet/handle/Lever',[
        (-.017,-.001,.006,.004),(-.015,.001,.012,.008),(-.008,.008,.018,.010),
        (.002,.014,.020,.009),(.015,.018,.0205,.006),(.030,.020,.022,.0045),
        (.060,.022,.025,.0038),(.086,.023,.025,.0038),(.096,.023,.022,.0034),
        (.100,.023,.016,.0028),(.102,.023,.005,.001)],chrome,'handle')
    for x,mat,name in [(-.005,red,'HotMark'),(.005,blue,'ColdMark')]:
        dot=UsdGeom.Sphere.Define(stage,'/Faucet/temperature_carrier/'+name);dot.CreateRadiusAttr(.0018)
        UsdGeom.Xformable(dot).AddTranslateOp().Set(Gf.Vec3d(x,.019,-.002))
        UsdShade.MaterialBindingAPI.Apply(dot.GetPrim()).Bind(mat)
    world=UsdPhysics.FixedJoint.Define(stage,'/Faucet/joints/world_fixed')
    world.CreateBody0Rel().SetTargets(['/Faucet/body'])
    world.CreateLocalPos0Attr(Gf.Vec3f(0));world.CreateLocalPos1Attr(Gf.Vec3f(0))
    hinge(stage,'temperature_yaw','/Faucet/body','/Faucet/temperature_carrier','Z',-45.,45.,PIVOT)
    hinge(stage,'flow_lift','/Faucet/temperature_carrier','/Faucet/handle','X',0.,35.,(0.,0.,0.))
    # Estimated solid inertias and nominal dimensions, not a manufacturer's CAD.
    stage.GetRootLayer().Save();stage.GetRootLayer().Export(str(ROOT/'single_lever_faucet.usd'))
    (ROOT/'tools'/'geometry.json').write_text(json.dumps(GEOMETRY),encoding='utf-8')
    demo=Usd.Stage.CreateNew(str(ROOT/'demo_scene.usda'))
    UsdGeom.SetStageMetersPerUnit(demo,1.);UsdGeom.SetStageUpAxis(demo,'Z')
    scene=UsdPhysics.Scene.Define(demo,'/World/PhysicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    scene.GetPrim().CreateAttribute('physxScene:timeStepsPerSecond',Sdf.ValueTypeNames.UInt).Set(120)
    demo.SetDefaultPrim(demo.GetPrimAtPath('/World'))
    demo.OverridePrim('/World/Faucet').GetReferences().AddReference('./single_lever_faucet.usd')
    counter=UsdGeom.Cube.Define(demo,'/World/Counter');counter.CreateSizeAttr(1.)
    xf=UsdGeom.Xformable(counter);xf.AddTranslateOp().Set(Gf.Vec3d(0,0,-.018));xf.AddScaleOp().Set(Gf.Vec3f(.55,.45,.036))
    UsdPhysics.CollisionAPI.Apply(counter.GetPrim())
    counter.GetDisplayColorAttr().Set([Gf.Vec3f(.16,.18,.20)])
    key=UsdLux.RectLight.Define(demo,'/World/KeyLight');key.CreateIntensityAttr(1800.);key.CreateWidthAttr(.7);key.CreateHeightAttr(.7)
    xf=UsdGeom.Xformable(key);xf.AddTranslateOp().Set(Gf.Vec3d(.2,.3,.8))
    fill=UsdLux.DomeLight.Define(demo,'/World/FillLight');fill.CreateIntensityAttr(600.)
    camera=UsdGeom.Camera.Define(demo,'/World/Camera');camera.CreateClippingRangeAttr(Gf.Vec2f(.001,100.));camera.CreateFocalLengthAttr(55.)
    view=Gf.Matrix4d().SetLookAt(Gf.Vec3d(.39,.50,.31),Gf.Vec3d(0,.035,.12),Gf.Vec3d(0,0,1)).GetInverse()
    UsdGeom.Xformable(camera).AddTransformOp().Set(view)
    demo.SetMetadata('customLayerData',{'cameraSettings':{'boundCamera':'/World/Camera'}})
    demo.GetRootLayer().Save()
    print('Created',ROOT/'single_lever_faucet.usd')

if __name__=='__main__':build()
