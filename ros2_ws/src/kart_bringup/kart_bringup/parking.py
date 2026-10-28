"""Parking composition: one reference owner, no automatic reverse recovery."""
from pathlib import Path
import yaml


def validate(root, site, slot='', recovery='', tuning=''):
    from kart_bringup.recovery import configuration
    from kart_parking.planner import Settings, load_site
    root=Path(root)
    if configuration(root/'recovery',recovery)[0]:
        raise ValueError('Parking prohibits reverse recovery; select recovery:=off')
    p=yaml.safe_load((root/'parking/parking.yaml').read_text())['/**/parking_planner']['ros__parameters']
    c=Settings(**{k:p[k] for k in vars(Settings())}).validate()
    load_site(site,p['map_frame'],slot or p['slot'],c)
    pp=yaml.safe_load((root/'control/pure_pursuit.yaml').read_text())['/**/pure_pursuit']['ros__parameters']
    for key in ('wheelbase_m','max_steering_rad'):
        if abs(p[key]-pp[key])>1e-8:
            raise ValueError('Parking/tracking vehicle geometry differs: '+key)
    if pp['parking_goal_tolerance_m']>=p['goal_position_m']:
        raise ValueError('Pursuit must stop inside parking goal position tolerance')


def node_action(root,site,slot='',use_sim_time=''):
    from launch.actions import LogInfo
    from launch_ros.actions import Node
    config=str(Path(root)/'parking/parking.yaml')
    overrides={'site_file':site}
    if slot: overrides['slot']=slot
    if use_sim_time!='':
        if str(use_sim_time).lower() not in ('true','false'): raise ValueError('Invalid use_sim_time')
        overrides['use_sim_time']=str(use_sim_time).lower()=='true'
    return [LogInfo(msg=f'Parking: {config}; explicit overrides: {overrides}'),
            Node(package='kart_parking',executable='parking_planner',name='parking_planner',
                 parameters=[config,overrides],output='screen')]
