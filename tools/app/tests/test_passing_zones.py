"""Studio persistence/export of permissions bound to immutable generated points."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from kart_studio import registration
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json, read_json
from kart_studio.passing_zones import geometry_revision


class PassingZoneTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.maps = Maps(Path(self.temp.name) / 'maps')
        self.folder = self.maps.root / 'course'
        self.folder.mkdir()
        document = dict(schema='kart.hdmap.v2', title='course', frame='map', revision=1, point_count=0,
                        updated=0, obstacles=[], lanes=[dict(id='main',closed=False,left=[[0,1],[12,1]],
                        right=[[0,-1],[12,-1]],custom=[[0,0],[12,0]],custom_speeds=[],lines={})])
        atomic_json(self.folder / 'map.json', document)
        atomic_json(self.folder / 'snapshot.json', dict(frame='map'))
        atomic_json(self.folder / 'cloud.json', dict(points=[],trajectory=[]))
        self.generate()

    def generate(self, spacing=.5):
        document = self.maps.load('course')
        self.maps.generate('course','centerline',dict(spacing=spacing),document['revision'],lambda:None,'main')

    @staticmethod
    def zone(line, **changes):
        value=dict(id='pass_001',start_index=1,end_index=5,side='left',geometry_revision=line['geometry_revision'])
        value.update(changes)
        return value

    def authorize(self):
        document = self.maps.load('course')
        line=document['lanes'][0]['lines']['centerline']
        line['passing_zones']=[self.zone(line)]
        return self.maps.save('course',document)

    def test_default_is_zero_and_loading_only_exposes_a_server_geometry_hash(self):
        before=(self.folder/'map.json').read_bytes()
        document=self.maps.load('course')
        line=document['lanes'][0]['lines']['centerline']
        self.assertEqual(line['passing_zones'],[])
        self.assertEqual(line['geometry_revision'],geometry_revision(line['points'],False))
        self.assertEqual((self.folder/'map.json').read_bytes(),before)

    def test_save_reload_export_yaml_metadata_and_ros_roundtrip_preserve_multiple_sides(self):
        document=self.authorize()
        line=document['lanes'][0]['lines']['centerline']
        line['passing_zones'].append(self.zone(line,id='pass_002',start_index=7,end_index=10,side='right'))
        line['passing_zones'].append(self.zone(line,id='pass_003',start_index=12,end_index=14,side='both'))
        document=self.maps.save('course',document)
        zones=document['lanes'][0]['lines']['centerline']['passing_zones']
        self.assertEqual(self.maps.load('course')['lanes'][0]['lines']['centerline']['passing_zones'],zones)
        exported=self.maps.export('course')
        self.assertEqual(read_json(exported/'hd_map.yaml')['lanes'][0]['line_passing_zones']['centerline'],zones)
        self.assertEqual(read_json(exported/'metadata.json')['lanes']['main']['lines']['centerline']['passing_zones'],zones)
        from kart_hdmap.document import load
        self.assertEqual(load(exported/'hd_map.yaml')['lanes'][0]['lines']['centerline']['passing_zones'],zones)

    def test_invalid_or_stale_zones_never_commit_and_generated_coordinates_remain_server_owned(self):
        document=self.authorize()
        for change in (dict(start_index=True),dict(end_index=1),dict(start_index=5,end_index=1),
                       dict(end_index=10000),dict(side='none'),dict(geometry_revision='0'*64),dict(extra=1)):
            bad=copy.deepcopy(document)
            bad['lanes'][0]['lines']['centerline']['passing_zones'][0].update(change)
            with self.subTest(change=change),self.assertRaises(ValueError):self.maps.save('course',bad)
            self.assertEqual(self.maps.load('course'),document)
        forged=copy.deepcopy(document)
        forged['lanes'][0]['lines']['centerline']['points']=[[99,99],[100,100]]
        saved=self.maps.save('course',forged)
        self.assertEqual(saved['lanes'][0]['lines']['centerline']['points'],document['lanes'][0]['lines']['centerline']['points'])

    def test_exact_regeneration_keeps_permissions_but_resampling_and_shape_edits_clear_with_notice(self):
        document=self.authorize()
        zones=document['lanes'][0]['lines']['centerline']['passing_zones']
        self.generate()
        self.assertEqual(self.maps.load('course')['lanes'][0]['lines']['centerline']['passing_zones'],zones)
        self.generate(.4)
        line=self.maps.load('course')['lanes'][0]['lines']['centerline']
        self.assertEqual(line['passing_zones'],[])
        self.assertIn('失効',line['passing_zones_invalidated'])
        document=self.authorize()
        document['lanes'][0]['left'][0][1]=1.1
        saved=self.maps.save('course',document)
        self.assertEqual(saved['lanes'][0]['lines'],{})
        self.assertIn('失効',saved['lanes'][0]['passing_zones_notice'])

    def test_registration_does_not_move_permissions_to_new_coordinates(self):
        source=self.authorize()
        atomic_json(self.maps.root/'target/map.json',source)
        body=dict(source='course',revision=source['revision'],x=2.,y=3.,yaw=10.)
        result=registration.preview(self.maps,'target',body)
        line=result['document']['lanes'][0]['lines']['centerline']
        self.assertEqual(line['passing_zones'],[])
        self.assertIn('失効',result['document']['lanes'][0]['passing_zones_notice'])
        self.assertTrue(self.maps.load('course')['lanes'][0]['lines']['centerline']['passing_zones'])

    def test_invalid_disk_annotations_remain_deletable_but_are_rejected_by_export(self):
        document=self.authorize()
        document['lanes'][0]['lines']['centerline']['points'][0][0]+=.01
        atomic_json(self.folder/'map.json',document)
        loaded=self.maps.load('course')
        line=loaded['lanes'][0]['lines']['centerline']
        self.assertIn('passing_zones_error',line)
        with self.assertRaises(ValueError):self.maps.export('course')
        line['passing_zones']=[]
        self.maps.save('course',loaded)
        self.assertEqual(self.maps.load('course')['lanes'][0]['lines']['centerline']['passing_zones'],[])


if __name__=='__main__':
    unittest.main()
