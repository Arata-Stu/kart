"""Synthetic histogram metadata; no pretrained model/accuracy claim."""

def metadata(width=160):
    height = width * 3 // 4
    return dict(
        format='eventstate_encoder_bundle_v1',
        runtime=dict(dinov3_repository='facebookresearch/dinov3:adc254450203739c8149213a7a69d8d905b4fcfa'),
        architecture=dict(architecture='dinov3_vits16'),
        input_shape=[1, 20, ((height+15)//16)*16, width],
        dataset=dict(sensor_height=480, sensor_width=640,
                     input_height=((height+15)//16)*16, input_width=width,
                     content_height=height, content_width=width, event_resize_mode='sum_pool',
                     representation=dict(temporal_interpolation='none', window_us=40000,
                                         event_bins=10, channel_layout='polarity_major',
                                         voxel_normalization='none', fixed_normalization=dict(
                                             event_resize_mode='sum_pool', content_size=[height, width],
                                             format='eventstate_fixed_normalization_v1',
                                             sensor_size=[480, 640], includes_zero_pixels=True,
                                             representation=dict(type='voxel_grid', channels=20, event_bins=10,
                                                                 polarity_split=True,
                                                                 channel_order='positive_0,...,positive_B-1,negative_0,...,negative_B-1',
                                                                 normalization='none', temporal_interpolation='none',
                                                                 histogram_version=2, window_us=40000,
                                                                 window_boundary='[start,end)'),
                                             mean=[0.1]*20, std=[2.0]*20))))
