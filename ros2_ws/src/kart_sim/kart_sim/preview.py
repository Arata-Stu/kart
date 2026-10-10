"""Standalone viewer / finite headless smoke; does not publish ROS commands."""
import argparse
from pathlib import Path
import time
import mujoco
from .core import Simulation

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--map',required=True,help='map JSON path')
    parser.add_argument('--assets',default=str(Path(__file__).resolve().parents[1]/'assets'))
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--render',help='Save an overview PNG with an offscreen GL context')
    parser.add_argument('--sensors',action='store_true',help='Render nominal D455 stereo/RGB and show a loopback browser monitor')
    parser.add_argument('--stereo-hz',type=float,default=60.)
    parser.add_argument('--rgb-hz',type=float,default=30.)
    parser.add_argument('--imu-hz',type=float,default=200.)
    parser.add_argument('--rig',help='D455 calibration JSON')
    args=parser.parse_args()
    sim=Simulation(args.map,args.assets,camera=args.sensors,rig_file=args.rig,
                   stereo_hz=args.stereo_hz,rgb_hz=args.rgb_hz,imu_hz=args.imu_hz)
    monitor=None
    try:
        if args.check:
            for _ in range(100): sim.step(steps=10,collect_sensors=args.sensors)
            print('map=',sim.map['name'],'geoms=',sim.model.ngeom,'time=',sim.data.time,'rear_axle=',sim.state()[0])
            if any(w.number for w in sim.data.warning): raise RuntimeError('MuJoCo warning')
        elif args.render:
            from .monitor import png
            renderer=mujoco.Renderer(sim.model,height=720,width=960)
            opt=mujoco.MjvOption(); opt.geomgroup[4]=0; opt.geomgroup[5]=1
            camera=mujoco.MjvCamera()
            a,b,c,d=sim.map['room']['bounds']
            camera.lookat[:]=[(a+c)/2,(b+d)/2,0]
            camera.distance=max(c-a,d-b)*1.05; camera.azimuth=90; camera.elevation=-80
            renderer.update_scene(sim.data,camera=camera,scene_option=opt)
            rgb=renderer.render()
            Path(args.render).write_bytes(png(rgb)); renderer.close()
        else:
            from mujoco import viewer as mjviewer
            steering,throttle=[0.0],[0.0]
            if args.sensors:
                from .monitor import SensorMonitor
                monitor=SensorMonitor()
            next_monitor=0.0
            def key(code):
                if code in (87,83): throttle[0]=max(-1,min(1,throttle[0]+(.05 if code==87 else -.05)))
                if code in (65,68): steering[0]=max(-1,min(1,steering[0]+(.1 if code==65 else -.1)))
                if code==88: steering[0]=throttle[0]=0
                if code==82: sim.reset(); steering[0]=throttle[0]=0
            with mjviewer.launch_passive(sim.model,sim.data,key_callback=key) as viewer:
                viewer.opt.geomgroup[4]=0; viewer.opt.geomgroup[5]=1
                a,b,c,d=sim.map['room']['bounds']
                viewer.cam.lookat[:]=[(a+c)/2,(b+d)/2,0]
                viewer.cam.distance=max(c-a,d-b); viewer.cam.elevation=-70
                while viewer.is_running():
                    start=time.monotonic()
                    sim.step(steering[0],max(throttle[0],0),reverse=max(-throttle[0],0),steps=20,collect_sensors=args.sensors)
                    if monitor and time.monotonic()>=next_monitor:
                        monitor.update(sim); next_monitor=time.monotonic()+.2
                    viewer.sync(); time.sleep(max(0,.02-(time.monotonic()-start)))
    finally:
        if monitor: monitor.close()
        sim.close()

if __name__=='__main__': main()
