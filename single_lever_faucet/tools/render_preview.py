"""Run with Blender --background --python this_file.py. Geometry is original."""
import bpy,json,pathlib,math
from mathutils import Vector,Matrix
ROOT=pathlib.Path(__file__).resolve().parents[1]
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
def mat(name,color,metallic=0.,roughness=.2):
    m=bpy.data.materials.new(name);m.use_nodes=True
    bsdf=next((n for n in m.node_tree.nodes if n.type=='BSDF_PRINCIPLED'),None)
    if bsdf is None:
        bsdf=m.node_tree.nodes.new('ShaderNodeBsdfPrincipled')
        output=next((n for n in m.node_tree.nodes if n.type=='OUTPUT_MATERIAL'),None) or m.node_tree.nodes.new('ShaderNodeOutputMaterial')
        m.node_tree.links.new(bsdf.outputs['BSDF'],output.inputs['Surface'])
    bsdf.inputs['Base Color'].default_value=(*color,1)
    bsdf.inputs['Metallic'].default_value=metallic;bsdf.inputs['Roughness'].default_value=roughness
    return m
chrome=mat('Chrome',(.65,.68,.72),1.,.15)
black=mat('Aerator',(.02,.02,.025),.1,.4)
objects=[]
for g in json.loads((ROOT/'tools'/'geometry.json').read_text()):
    mesh=bpy.data.meshes.new(g['path']);mesh.from_pydata(g['points'],[],g['faces']);mesh.update()
    obj=bpy.data.objects.new(g['path'],mesh);bpy.context.collection.objects.link(obj)
    obj.data.materials.append(chrome)
    for face in mesh.polygons:face.use_smooth=True
    objects.append((obj,g['link']))
bpy.ops.mesh.primitive_cylinder_add(vertices=64,radius=.012,depth=.002,location=(0,.116,.126))
bpy.context.object.data.materials.append(black)
for x,color in [(-.005,(.6,.015,.01)),(.005,(.01,.1,.6))]:
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=12,radius=.0018,location=(x,.019,-.002))
    obj=bpy.context.object;obj['local_position']=tuple(obj.location);obj.data.materials.append(mat(str(color),color));objects.append((obj,'temperature_carrier'))
floor=mat('Counter',(.095,.11,.13),.1,.28)
bpy.ops.mesh.primitive_cube_add(size=1,location=(0,0,-.018));obj=bpy.context.object;obj.scale=(.65,.65,.036);obj.data.materials.append(floor)
bevel=obj.modifiers.new('Edge softness','BEVEL');bevel.width=.006;bevel.segments=3
def light(name,pos,power,size,scale=(1,1,1)):
    data=bpy.data.lights.new(name,'AREA');data.energy=power;data.shape='RECTANGLE';data.size=size;data.size_y=size*.6
    obj=bpy.data.objects.new(name,data);bpy.context.collection.objects.link(obj);obj.location=pos;obj.rotation_euler=(Vector((0,.03,.12))-obj.location).to_track_quat('-Z','Y').to_euler()
light('Large softbox',(.15,.3,.65),50,.5)
light('Vertical edge',(-.3,-.05,.3),35,.35)
light('Front strip',(.4,.15,.25),20,.22)
scene=bpy.context.scene;scene.world.color=(.3,.3,.3)
bpy.ops.object.camera_add(location=(.35,.52,.29));cam=bpy.context.object;cam.rotation_euler=(Vector((0,.035,.115))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.type='ORTHO';cam.data.ortho_scale=.31;scene.camera=cam
scene.render.engine='CYCLES';scene.cycles.samples=48
scene.render.resolution_x=900;scene.render.resolution_y=1000;scene.render.resolution_percentage=100
scene.view_settings.view_transform='AgX'
pivot=Matrix.Translation((0,0,.192))
for name,yaw,lift in [('preview_closed',0,0),('preview_open',-22,30)]:
    for obj,link in objects:
        local=Matrix.Identity(4)
        if link=='temperature_carrier':local=pivot@Matrix.Rotation(math.radians(yaw),4,'Z')
        if link=='handle':local=pivot@Matrix.Rotation(math.radians(yaw),4,'Z')@Matrix.Rotation(math.radians(lift),4,'X')
        if 'local_position' in obj:
            obj.matrix_world=local@Matrix.Translation(obj['local_position'])
        else:obj.matrix_world=local
    scene.render.filepath=str(ROOT/(name+'.png'));bpy.ops.render.render(write_still=True)
