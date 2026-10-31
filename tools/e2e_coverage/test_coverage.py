import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from core import compare, distribution, settings, status
import cli


def record(name,role='training',steering=.2,speed=1.,curvature=.1):
    return dict(id=name,role=role,rows=[dict(stamp_ns=1_000_000_000+i*100_000_000,
        steering=steering,speed=speed,curvature=curvature,valid=True,reason='',mode=2,artifact=None) for i in range(41)])


class CoverageTest(unittest.TestCase):
    def test_density_is_not_independent_support(self):
        cfg=settings({})
        one=distribution([record('one')],'steering',cfg)
        self.assertEqual(status(next(iter(one.values())),cfg),'sparse')
        many=distribution([record(str(i)) for i in range(3)],'steering',cfg)
        cell=next(iter(many.values()))
        self.assertEqual(status(cell,cfg),'observed_support')
        self.assertAlmostEqual(cell['seconds'],12.)

    def test_gap_invalid_and_mode_split(self):
        r=record('r');r['rows'][10]['valid']=False
        for row in r['rows'][20:]: row['stamp_ns']+=10_000_000_000
        for row in r['rows'][30:]: row['mode']=1
        cells=distribution([r],'steering',settings({}))
        cell=next(iter(cells.values()))
        self.assertAlmostEqual(cell['seconds'],3.6)
        self.assertEqual(cell['visits'],4)

    def test_event_only_before_and_unknown_preserved(self):
        baseline=[record(str(i)) for i in range(3)]
        evaluation=record('eval','evaluation')
        evaluation['events']=[dict(name='collision',stamp_ns=3_000_000_000,before_s=1.)]
        evaluation['rows'][10]['valid']=False
        for row in evaluation['rows'][20:]: row['steering']=-.9
        result=compare(baseline+[evaluation],settings({}))
        event=next(r for r in result['comparisons'] if r['window']=='collision' and r['baseline']=='training' and r['dimension']=='steering')
        self.assertEqual(event['samples'],10)
        self.assertEqual(event['unknown'],1)
        self.assertEqual(event['observed_support'],9)
        self.assertEqual(event['unobserved'],0)

    def test_geometry_and_command_are_separate(self):
        evaluation=record('eval','evaluation',steering=-.9)
        result=compare([record(str(i)) for i in range(3)]+[evaluation],settings({}))
        rows={r['dimension']:r for r in result['comparisons'] if r['baseline']=='training'}
        self.assertEqual(rows['steering']['unobserved'],41)
        self.assertEqual(rows['curvature_speed']['observed_support'],41)

    def test_missing_pose_teacher_is_unknown(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'image.png').write_bytes(b'x')
            (root/'metadata.json').write_text('{}')
            (root/'samples.jsonl').write_text(json.dumps(dict(stamp_ns=9_000_000_000,steering=.1,image='image.png'))+'\n')
            rows,_=cli.teacher_rows(root,record('a')['rows'],.05)
            self.assertFalse(rows[0]['valid'])
            self.assertEqual(rows[0]['reason'],'pose_unmatched')

    def test_offline_runner_and_reuse(self):
        with tempfile.TemporaryDirectory() as tmp:
            work=Path(tmp); bag=work/'bag'
            def execute(command, **kwargs):
                self.assertEqual(command[1:3],['-m','kart_mapping.build_map'])
                self.assertEqual(kwargs['env']['ROS_DOMAIN_ID'],'92')
                poses=work/'official'/'run'/'poses';poses.mkdir(parents=True)
                (poses/'odom_poses.tum').write_text('pose fixture')
            workflow=cli.ROOT/'ros2_ws/src/kart_bringup/config/mapping/workflow.json'
            with patch.object(cli.subprocess,'run',side_effect=execute) as called:
                pose,frame=cli.poses_for(dict(id='a'),bag,work,work,workflow)
                self.assertEqual(frame,'camera_link');self.assertEqual(called.call_count,1)
                self.assertEqual(cli.poses_for(dict(id='a',poses=str(pose),pose_frame=frame),bag,work,work,workflow),(pose,frame))
                self.assertEqual(called.call_count,1)

    def test_failure_preserves_output_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);manifest=root/'manifest.json';output=root/'out'
            manifest.write_text(json.dumps(dict(recordings=[dict(id='a',role='training',mode='manual',bag='missing',dataset='data')])))
            with self.assertRaises(Exception): cli.run(manifest,output)
            self.assertEqual(json.loads((output/'status.json').read_text())['status'],'failed')
            with self.assertRaises(FileExistsError): cli.run(manifest,output)

    def test_invalid_settings(self):
        for value in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError): settings(dict(speed_bin=value))

    def test_complete_report_with_recorded_poses_and_teacher_samples(self):
        from test_tuning import synthetic
        from decimal import Decimal
        data=synthetic()
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp).resolve();entries=[]
            for role in ('training','reference','evaluation'):
                folder=root/role;folder.mkdir();bag=folder/'bag';bag.mkdir()
                (bag/'metadata.yaml').write_text(role)
                pose=folder/'odom_poses.tum'
                origin=data['odom'][0]['header_ns']
                pose.write_text(''.join(f"{Decimal(r['header_ns'])/10**9} {(r['header_ns']-origin)/1e9} 0 0 0 0 0 1\n" for r in data['odom']))
                entry=dict(id=role,role=role,mode='manual',bag=str(bag),poses=str(pose),pose_frame='base_link')
                if role=='training':
                    dataset=folder/'dataset';dataset.mkdir()
                    (dataset/'image.png').write_bytes(b'fixture')
                    (dataset/'metadata.json').write_text('{}')
                    (dataset/'samples.jsonl').write_text(''.join(json.dumps(dict(stamp_ns=r['header_ns'],steering=0.,image='image.png'))+'\n' for r in data['odom']))
                    entry['dataset']=str(dataset)
                if role=='evaluation': entry['events']=[dict(name='collision',stamp_ns=origin+3_000_000_000)]
                entries.append(entry)
            manifest=root/'manifest.json';manifest.write_text(json.dumps(dict(recordings=entries)))
            with patch.object(cli,'read_bag',return_value=data):
                result=cli.run(manifest,root/'out')
            self.assertTrue((root/'out/report.html').is_file())
            self.assertEqual(json.loads((root/'out/status.json').read_text())['status'],'complete')
            self.assertEqual(result['events'][0]['name'],'collision')
            self.assertGreater(result['events'][0]['valid_samples'],0)
