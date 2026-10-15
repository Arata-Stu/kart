import copy
import unittest
from kart_e2e.evs_contract import validate_metadata


def metadata(size):
    h=size*3//4
    return dict(format='eventstate_encoder_bundle_v1',architecture=dict(architecture='dinov3_vits16'),
        input_shape=[1,20,((h+15)//16)*16,size],dataset=dict(input_height=((h+15)//16)*16,input_width=size,
        content_height=h,content_width=size,event_resize_mode='sum_pool',representation=dict(
        temporal_interpolation='none',window_us=40000,event_bins=10,channel_layout='polarity_major',
        voxel_normalization='none',fixed_normalization=dict(event_resize_mode='sum_pool',content_size=[h,size]))))


class HistogramContractTests(unittest.TestCase):
    def test_shapes(self):
        for size in (160,320):
            self.assertEqual(validate_metadata(metadata(size)),(size*3//4,size))

    def test_reject_linear_and_wrong_stats(self):
        for key,value in [('temporal_interpolation','linear'),('window_us',50000),('channel_layout','time_major')]:
            m=metadata(160);m['dataset']['representation'][key]=value
            with self.assertRaises(ValueError):validate_metadata(m)
        m=metadata(160)
        m['dataset']['representation']['fixed_normalization']['content_size']=[240,320]
        with self.assertRaises(ValueError):validate_metadata(m)
