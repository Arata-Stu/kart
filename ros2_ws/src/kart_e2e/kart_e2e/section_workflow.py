"""Trusted UI job worker for routing snapshots and finite offline localization runs."""
import argparse
import json
import shutil
import subprocess
from pathlib import Path
from .sections import Router, digest, validate_maps


def run(request,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    if request['action']=='routing':
        routing=request['routing'];router=Router(routing)
        # Recheck the selected map snapshot after the UI job acquired its resource lock.
        validate_maps(routing,routing['map_file'],Path(routing['map_file']).parent)
        (output/'routing.json').write_text(json.dumps(routing,indent=2))
        meta=dict(routing=routing,routing_sha256=router.sha)
    elif request['action']=='offline':
        routing=json.loads(Path(request['routing_file']).read_text());Router(routing)
        validate_maps(routing,request['map_file'],request['map_dir'])
        ros=shutil.which('ros2')
        if not ros:raise ValueError('ros2 is required for offline localization')
        marker=output/'playback_exit.json'
        command=[ros,'launch','kart_bringup','e2e_sections_offline.launch.py',
                 *[f'{key}:={request[key]}' for key in ('bag','map_file','map_dir','routing_file','tracking_mode')],
                 f'trajectory_output:={output/"poses.jsonl"}','exit_when_done:=true',f'completion_file:={marker}']
        print('Offline VSLAM: '+' '.join(command),flush=True)
        subprocess.run(command,check=True)
        if not marker.is_file() or json.loads(marker.read_text()).get('returncode')!=0:
            raise ValueError('Bag playback did not finish successfully; poses were not published')
        poses=[json.loads(line) for line in (output/'poses.jsonl').read_text().splitlines()]
        good=sum(bool(p.get('valid')) for p in poses)
        if not good:raise ValueError('No registered-map poses: check existing-map localization and image-time TF')
        meta=dict(bag=request['bag'],routing_sha256=digest(routing),valid_poses=good,total_poses=len(poses))
    else:raise ValueError('Unknown section workflow action')
    (output/'metadata.json').write_text(json.dumps(meta,indent=2))
    return meta


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(run(json.loads(args.request.read_text()),args.output),indent=2))

if __name__=='__main__':main()
