"""Single-lever mixer control. Use in Isaac Sim's Script Editor or Python.

USD drive targets use DEGREES; measured articulation positions use RADIANS.
All water states are idealized task variables, not liquid/heat simulation.
"""
import math
from pxr import UsdPhysics

def state_from_positions(temperature_yaw_rad, flow_lift_rad,
                         cold_c=20., hot_c=60., max_flow_lpm=6.):
    """Pass measured joint positions by NAME, independent of DOF array order."""
    if hot_c<=cold_c or max_flow_lpm<=0:raise ValueError('Invalid supply parameters')
    opening=min(1.,max(0.,flow_lift_rad/math.radians(35.)))
    mix=min(1.,max(0.,(temperature_yaw_rad+math.pi/4)/(math.pi/2)))
    return {'opening':opening,'flow_lpm':max_flow_lpm*opening,
            'flowing':opening>1e-4,
            'temperature_c':cold_c+mix*(hot_c-cold_c) if opening>1e-4 else None,
            'hot_fraction':mix}

def set_handle(stage, opening=0., temperature_c=40., root_path='/Faucet'):
    """Command both DOFs. Start Play for PhysX to move the handle.

    opening: 0..1; supply limits default 20..60 C. Returns TARGET state.
    """
    if not 0.<=opening<=1.:raise ValueError('opening must be between 0 and 1')
    root=stage.GetPrimAtPath(root_path)
    if not root:raise ValueError('Root not found: '+root_path)
    cold=root.GetAttribute('faucet:coldSupplyC').Get()
    hot=root.GetAttribute('faucet:hotSupplyC').Get()
    if hot<=cold:raise ValueError('hotSupplyC must exceed coldSupplyC')
    temperature_c=min(hot,max(cold,float(temperature_c)))
    yaw=-45.+90.*(temperature_c-cold)/(hot-cold)
    for name,degrees in [('temperature_yaw',yaw),('flow_lift',opening*35.)]:
        p=stage.GetPrimAtPath(root_path+'/joints/'+name)
        if not p:raise ValueError('Joint not found: '+name)
        UsdPhysics.DriveAPI.Get(p,'angular').GetTargetPositionAttr().Set(degrees)
    root.GetAttribute('faucet:targetOpening').Set(float(opening))
    root.GetAttribute('faucet:targetTemperatureC').Set(temperature_c)
    result=state_from_positions(math.radians(yaw),math.radians(opening*35.),cold,hot,
                               root.GetAttribute('faucet:maxFlowLpm').Get())
    result['kind']='target; use measured joint positions for actual state'
    return result

def set_passive(stage, enabled=True, root_path='/Faucet'):
    """Disable position holding for robot-contact manipulation; retain mild damping."""
    for name in ['temperature_yaw','flow_lift']:
        p=stage.GetPrimAtPath(root_path+'/joints/'+name)
        if not p:raise ValueError('Joint not found: '+name)
        d=UsdPhysics.DriveAPI.Get(p,'angular')
        d.GetStiffnessAttr().Set(0. if enabled else .06)
        d.GetDampingAttr().Set(.0005 if enabled else .005)

def show_panel(root_path='/Faucet'):
    """Optional live sliders in Isaac Sim. Keep the returned window alive."""
    import omni.ui as ui
    import omni.usd
    window=ui.Window('Single lever faucet',width=380,height=220)
    with window.frame:
        with ui.VStack(spacing=8):
            ui.Label('Lift = flow | Left/right = temperature')
            ui.Label('Opening (0 = closed, 1 = fully open)')
            opening=ui.FloatSlider(min=0.,max=1.)
            ui.Label('Temperature (20..60 C)')
            temperature=ui.FloatSlider(min=20.,max=60.)
            temperature.model.set_value(40.)
            status=ui.Label('Press Play to simulate handle motion.')
            def update(_):
                result=set_handle(omni.usd.get_context().get_stage(),opening.model.as_float,
                                  temperature.model.as_float,root_path)
                status.text=f"Target: {result['flow_lpm']:.1f} L/min | {result['temperature_c']} C"
            opening.model.add_value_changed_fn(update)
            temperature.model.add_value_changed_fn(update)
    return window
