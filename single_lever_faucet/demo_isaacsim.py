"""Run with Isaac Sim's python.bat, e.g. python.bat demo_isaacsim.py.

Designed for Isaac Sim 4.5/5.x; requires an installed Isaac Sim environment.
"""
from isaacsim import SimulationApp
app=SimulationApp({'headless':False})
import pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import omni.usd,omni.timeline
from control import show_panel,set_handle
context=omni.usd.get_context()
context.open_stage(str(ROOT/'demo_scene.usda'))
for _ in range(30):app.update()
try:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().set_active_camera('/World/Camera')
except (ImportError,AttributeError):pass
panel=show_panel('/World/Faucet')
timeline=omni.timeline.get_timeline_interface();timeline.play()
set_handle(context.get_stage(),0.,40.,'/World/Faucet')
try:
    while app.is_running():app.update()
finally:
    timeline.stop();app.close()
