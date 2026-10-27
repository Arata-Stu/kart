"""Plan from a saved HDMap without ROS, TF, services or actuator output."""
import argparse
import json
from pathlib import Path
import yaml
from kart_hdmap.document import load, select
from .planner import Settings, load_site, prepare, activate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map-file',required=True)
    p.add_argument('--lane-id',required=True)
    p.add_argument('--line-type',choices=('centerline','raceline','customline'),default='centerline')
    p.add_argument('--site',required=True)
    p.add_argument('--slot',required=True)
    p.add_argument('--config',required=True,help='Operational parking.yaml')
    p.add_argument('--start',nargs=3,type=float,required=True,metavar=('X','Y','YAW'))
    p.add_argument('--output',required=True,help='New plan JSON file; never overwrites')
    args=p.parse_args()
    values=yaml.safe_load(Path(args.config).read_text())['/**/parking_planner']['ros__parameters']
    c=Settings(**{k:values[k] for k in vars(Settings())}).validate()
    frame=values['map_frame']
    world,zone=load_site(args.site,frame,args.slot,c)
    lane=select(load(args.map_file,frame),args.lane_id)
    line=lane['lines'][args.line_type]
    speeds=([lane['centerline_speed_mps']]*len(line['points']) if args.line_type=='centerline'
            else [row[5] for row in line['profile']])
    prepared=prepare(line['points'],speeds,lane['closed'],zone,world,c)
    route=activate(prepared,tuple(args.start),c)
    result=dict(schema='kart.parking.plan.v1',frame_id=frame,slot=args.slot,
                goal=zone['goal'],departure=prepared.course[prepared.departure][:3],
                columns=['x_m','y_m','yaw_rad','speed_mps'],route=route,
                validation='static geometry only; tracking/localization/physical stop unverified')
    with Path(args.output).open('x') as f:
        json.dump(result,f,indent=2,allow_nan=False); f.write('\n')
    print(f'Saved {len(route)} samples: {args.output}')


if __name__=='__main__': main()
