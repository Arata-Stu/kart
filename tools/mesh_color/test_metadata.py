import copy
import unittest
from metadata import pose_reference


class MetadataTests(unittest.TestCase):
    def fixture(self):
        return dict(camera_params_id_to_camera_params={
            '0': dict(sensor_meta_data=dict(sensor_to_vehicle_transform=dict(axis_angle={}, translation={}))),
            '1': dict(sensor_meta_data=dict(sensor_to_vehicle_transform=dict(axis_angle={}, translation={})))},
            keyframes_metadata=[dict(camera_params_id='0',camera_to_world={},timestamp_microseconds='1'),
                                dict(camera_params_id='1',camera_to_world={},timestamp_microseconds='1')])

    def test_no_stereo_pair_or_unknown_keys(self):
        meta=self.fixture()
        self.assertEqual(pose_reference(meta)[0], '0')
        meta['stereo_pair']=[{'different_schema':123}]
        self.assertEqual(pose_reference(meta)[0], '0')

    def test_reference_and_extrinsic_are_same_camera(self):
        meta=self.fixture()
        meta['keyframes_metadata'].append(dict(camera_params_id='1',camera_to_world={},timestamp_microseconds='2'))
        key,cal,frames=pose_reference(meta)
        self.assertEqual(key,'1')
        self.assertIs(cal,meta['camera_params_id_to_camera_params']['1'])
        self.assertEqual({f['camera_params_id'] for f in frames},{key})

    def test_missing_extrinsics_no_identity_fallback(self):
        meta=self.fixture()
        for cal in meta['camera_params_id_to_camera_params'].values():
            cal['sensor_meta_data']={}
        with self.assertRaisesRegex(ValueError,'No camera'):
            pose_reference(meta)

    def test_incomplete_pose_rejected(self):
        meta=self.fixture()
        del meta['keyframes_metadata'][0]['timestamp_microseconds']
        with self.assertRaisesRegex(ValueError,'incomplete'):
            pose_reference(meta)
