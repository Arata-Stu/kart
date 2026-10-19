"""RAW adapter, causal teachers, real DINOv3 training and ONNX contract tests."""

import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from evs_fixtures import metadata
from kart_e2e.contract import evs_spec, input_contract, model_contract

HAS_NUMPY = importlib.util.find_spec('numpy') is not None
HAS_TORCH = importlib.util.find_spec('torch') is not None
REPO = os.environ.get('KART_DINOV3_REPO')


@unittest.skipUnless(HAS_NUMPY, 'numpy required')
class RawDiscoveryTests(unittest.TestCase):
    def test_single_raw_identity_and_ambiguous_sources(self):
        from kart_e2e.preprocess_evs import discover_raw
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory)
            timing = [dict(stream_id='camera', sensor_epoch=3)]
            with self.assertRaisesRegex(ValueError, '0件'):
                discover_raw(bag, timing)
            raw = bag / 'events.RAW'; raw.write_bytes(b'raw')
            result = discover_raw(bag, timing * 2)
            self.assertEqual(result, [dict(path=str(raw.resolve()),stream_id='camera',sensor_epoch=3)])
            for rows in ([], timing + [dict(stream_id='camera',sensor_epoch=4)],
                         timing + [dict(stream_id='other',sensor_epoch=3)]):
                with self.assertRaisesRegex(ValueError, '一意'):
                    discover_raw(bag, rows)
            other = bag / 'other.raw'; other.write_bytes(b'raw')
            with self.assertRaisesRegex(ValueError, '2件'):
                discover_raw(bag, timing)
            other.unlink(); raw.write_bytes(b'')
            with self.assertRaisesRegex(ValueError, '空ファイル'):
                discover_raw(bag, timing)
            raw.unlink(); raw.symlink_to(bag / 'metadata.yaml')
            with self.assertRaisesRegex(ValueError, 'シンボリックリンク'):
                discover_raw(bag, timing)


class MetadataTests(unittest.TestCase):
    def test_resolution_normalization_and_version_fail_closed(self):
        for width in (160, 320):
            self.assertEqual(evs_spec(metadata(width))['width'], width)
        for change in ('version', 'stats', 'mean', 'std', 'sensor', 'shape'):
            meta = metadata()
            rep = meta['dataset']['representation']
            if change == 'version':
                rep['fixed_normalization']['representation']['histogram_version'] = 1
            elif change == 'stats':
                rep['fixed_normalization']['content_size'] = [240, 320]
            elif change == 'mean':
                rep['fixed_normalization']['mean'][0] = float('nan')
            elif change == 'std':
                rep['fixed_normalization']['std'][0] = 0
            elif change == 'sensor':
                meta['dataset']['sensor_width'] = 641
            else:
                meta['input_shape'] = [1, 3, 120, 160]
            with self.assertRaises(ValueError, msg=change):
                evs_spec(meta)

    def test_onnx_binding_and_hash_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'model.onnx').write_bytes(b'fixture')
            meta = metadata()
            data = dict(schema=1, modality='evs', model_spec=evs_spec(meta), evs_metadata=meta,
                        output_mode='steer_only', input_name='input_tensor',
                        input_shape=[1, 20, 120, 160], output_name='control', output_shape=[1, 1],
                        onnx_sha256=hashlib.sha256(b'fixture').hexdigest())
            path = root / 'metadata.json'
            path.write_text(json.dumps(data))
            self.assertEqual(input_contract(root), ('evs', 'input_tensor', [1, 20, 120, 160]))
            for key, value in (('input_name', 'image'), ('input_shape', [1, 20, 128, 160]),
                               ('onnx_sha256', 'bad'), ('modality', 'rgb')):
                path.write_text(json.dumps(dict(data, **{key: value})))
                with self.assertRaises(ValueError, msg=key):
                    model_contract(root)


@unittest.skipUnless(HAS_NUMPY, 'numpy required')
class RawTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.dtype = [('x', '<u2'), ('y', '<u2'), ('p', 'u1'), ('t', '<i8')]
        self.events = np.array([(0, 0, 1, 0), (3, 3, 1, 0), (4, 4, 0, 4000),
                                (639, 479, 0, 39999), (0, 0, 1, 40000),
                                (0, 0, 1, 50000)], dtype=self.dtype)
        self.offset = 1_700_000_000_000_000_123

    def timing(self, seq=0, start=0, epoch=0):
        return dict(schema='openeb.tensor_timing.v1', stream_id='camera-stream', sequence=seq,
                    sensor_epoch=epoch, tensor_stamp_ros_ns=(start+40000)*1000+self.offset,
                    window_start_sensor_us=start, window_end_sensor_us=start+40000,
                    sensor_to_ros_offset_ns=self.offset, latest_event_sensor_us=start+40000,
                    window_start_ros_ns=start*1000+self.offset,
                    window_end_ros_ns=(start+40000)*1000+self.offset, shape=[1, 20, 120, 212])

    def reader(self, chunks=None):
        chunks = chunks or [self.events[:3], self.events[3:]]
        class FakeReader:
            def __init__(reader, path, do_time_shifting):
                self.assertFalse(do_time_shifting)
                reader.chunks = [c.copy() for c in chunks]
            def get_size(reader):
                return 480, 640
            def is_done(reader):
                return not reader.chunks
            def load_n_events(reader, count):
                return reader.chunks.pop(0)
        return FakeReader

    def test_histogram_boundaries_polarity_pooling_empty_and_cutoff(self):
        import numpy as np
        from kart_e2e.preprocess_evs import histogram
        counts = histogram(self.events, 0, 40000, (480, 640), (120, 160))
        self.assertEqual(counts.dtype, np.float32)
        self.assertEqual(counts[0, 0, 0], 2)
        self.assertEqual(counts[11, 1, 1], 1)
        self.assertEqual(counts[19, 119, 159], 1)
        self.assertEqual(counts.sum(), 4)
        self.assertEqual(histogram(self.events[:0], 0, 40000, (480, 640), (120, 160)).sum(), 0)
        self.assertEqual(histogram(self.events, 0, 40000, (480, 640), (120, 160), 4000).sum(), 3)
        bad = self.events.copy()
        bad[0]['x'] = 640
        with self.assertRaises(ValueError):
            histogram(bad, 0, 40000, (480, 640), (120, 160))

    def test_streaming_edges_and_reset(self):
        from kart_e2e.preprocess_evs import RawWindows
        raw = RawWindows('fake.raw', self.reader())
        self.assertEqual(len(raw.window(0, 40000)), 4)
        self.assertIsNone(raw.window(20000, 60000))
        with self.assertRaises(ValueError):
            raw.window(0, 40000)
        resetting = RawWindows('fake.raw', self.reader([self.events, self.events]))
        with self.assertRaisesRegex(ValueError, 'reset'):
            resetting.window(0, 60000)

    def test_numpy2_rejected_before_native_reader_import(self):
        from kart_e2e.preprocess_evs import RawWindows
        with patch('kart_e2e.preprocess_evs.np.__version__', '2.5.4'):
            with self.assertRaisesRegex(ValueError, 'NumPy 1.26'):
                RawWindows('not-opened.raw')

    def test_raw_regression_reports_magnitude_and_position(self):
        from kart_e2e.preprocess_evs import RawWindows
        chunk = self.events.copy()
        chunk['t'] = [0, 100, 99, 200, 40000, 50000]
        raw = RawWindows('diagnostic.raw', self.reader([chunk]))
        with self.assertRaisesRegex(ValueError, r'event_index=2; location=within_chunk; previous_us=100; current_us=99; backward_us=1;'):
            raw.window(0, 40000)

    def test_causal_labels_epoch_mapping_atomic_output_and_gaps(self):
        import numpy as np
        from kart_e2e.preprocess_evs import extract
        row = self.timing()
        stamp = row['tensor_stamp_ros_ns']
        teacher = types.SimpleNamespace(steering=0.2, throttle=0.1, brake=0, reverse=0)
        future = types.SimpleNamespace(steering=0.9, throttle=0.9, brake=0, reverse=0)
        commands = [(stamp-1, teacher), (stamp+1, future)]
        modes = [(stamp-1, types.SimpleNamespace(mode=2))]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            raw = path / 'camera.raw'
            raw.write_bytes(b'RAW adapter fixture')
            files = [dict(path=str(raw), stream_id='camera-stream', sensor_epoch=0)]
            output = path / 'dataset'
            report = extract(files, commands, modes, [row, self.timing(2, 4000)], metadata(),
                             output, sample_stride=1, reader_factory=self.reader(), provenance={"clock":"header"})
            self.assertEqual(report['samples'], 2)
            self.assertEqual(report['extraction'], {'clock':'header'})
            self.assertEqual(report['timing_sequence_gaps'], 1)
            sample = json.loads((output / 'samples.jsonl').read_text().splitlines()[0])
            self.assertEqual(sample['stamp_ns'], stamp)
            self.assertEqual(sample['steering'], 0.2)
            self.assertEqual(np.load(output / sample['counts']).sum(), 4)
            for bad_timings in ([row, row], [self.timing(epoch=1)],
                                [dict(row, window_end_ros_ns=stamp+1)]):
                target = path / 'bad'
                with self.assertRaises(ValueError):
                    extract(files, commands, modes, bad_timings, metadata(), target,
                            reader_factory=self.reader())
                self.assertFalse(target.exists())
            with self.assertRaises(ValueError):
                extract(files, commands, [(stamp-1, types.SimpleNamespace(mode=1))],
                        [row], metadata(), path / 'no_teacher', reader_factory=self.reader())
            self.assertFalse((path / 'no_teacher').exists())
            # Fixed-intended bags use only AUTO human steering; MANUAL is recovery.
            from kart_e2e.teacher_selection import AUTO_SCHEMA
            fixed = dict(schema=AUTO_SCHEMA, enabled=True, armed=True, throttle=0.1,
                         speed_scale=0.3, entry_neutral_s=0.3, mode=1)
            auto_modes = [(stamp-400_000_000+i*100_000_000, types.SimpleNamespace(mode=1))
                          for i in range(5)]
            auto_modes.append((stamp+1, types.SimpleNamespace(mode=2)))
            fixed_output = path / 'fixed_dataset'
            report = extract(files, commands, auto_modes, [row, self.timing(2, 4000)], metadata(),
                             fixed_output, sample_stride=1, reader_factory=self.reader(),
                             fixed_configs=[(stamp-1, fixed)])
            self.assertEqual(report['samples'], 1)
            self.assertEqual(report['teacher_selection']['policy'], 'fixed_auto_only')
            sample = json.loads((fixed_output / 'samples.jsonl').read_text())
            self.assertEqual(sample['steering'], 0.2)
            self.assertEqual(sample['teacher_source'], 'fixed_auto')
            # Ambiguous overlapping split files must never duplicate samples.
            with self.assertRaisesRegex(ValueError, 'Overlapping'):
                extract(files+files, commands, modes, [row], metadata(), path / 'overlap',
                        sample_stride=1, reader_factory=self.reader())

    def test_bag_adapter_uses_header_for_teacher_and_integer_timing(self):
        from kart_e2e.preprocess_evs import bag_records
        row = self.timing()
        stamp = row['tensor_stamp_ros_ns']
        header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=stamp//10**9, nanosec=stamp%10**9))
        command = types.SimpleNamespace(header=header)
        mode = types.SimpleNamespace(header=header)
        timing = types.SimpleNamespace(data=json.dumps(row))
        names = ['/cmd', '/mode', '/timing']
        msgs = [command, mode, timing]
        class Reader:
            def open(self, *args): pass
            def get_all_topics_and_types(self):
                return [types.SimpleNamespace(name=n, type=t) for n, t in zip(names,
                        ['kart_interfaces/msg/ControlCommand', 'kart_interfaces/msg/OperationModeState', 'std_msgs/msg/String'])]
            def set_filter(self, *args): self.index = 0
            def has_next(self): return self.index < 3
            def read_next(self):
                i = self.index
                self.index += 1
                return names[i], msgs[i], stamp+5000000
        modules = {n: types.ModuleType(n) for n in ('rosbag2_py', 'rclpy.serialization', 'rosidl_runtime_py.utilities')}
        modules['rosbag2_py'].SequentialReader = Reader
        for key in ('StorageOptions', 'ConverterOptions', 'StorageFilter'):
            setattr(modules['rosbag2_py'], key, lambda *a, **k: None)
        modules['rclpy.serialization'].deserialize_message = lambda raw, _: raw
        modules['rosidl_runtime_py.utilities'].get_message = lambda _: None
        with patch.dict(sys.modules, modules):
            commands, modes, timings = bag_records('bag', *names, 'header')
        self.assertEqual(commands[0][0], stamp)
        self.assertEqual(modes[0][0], stamp)
        self.assertEqual(timings, [row])


@unittest.skipUnless(REPO and HAS_TORCH and importlib.util.find_spec('onnxruntime'),
                     'KART_DINOV3_REPO + torch/ONNX required')
class RealPolicyTests(unittest.TestCase):
    def test_train_checkpoint_export_and_evaluation_both_heads(self):
        import numpy as np
        import torch
        from torch import nn
        from kart_e2e.model import backbone
        from kart_e2e.evs_policy import load_checkpoint
        from kart_e2e.export_onnx import export
        from kart_e2e.evaluate import evaluate
        torch.set_num_threads(2)
        torch.manual_seed(42)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            meta = metadata()
            encoder = backbone(REPO)
            encoder.patch_embed.proj = nn.Conv2d(20, 384, 16, stride=16)
            weights = root / 'encoder.pt'
            torch.save(dict(metadata=meta, state_dict=encoder.state_dict()), weights)
            dataset = root / 'dataset'
            dataset.mkdir()
            np.save(dataset / 'counts.npy', np.zeros((20, 120, 160), dtype=np.float32))
            (dataset / 'metadata.json').write_text(json.dumps(dict(modality='evs', model_spec=evs_spec(meta))))
            (dataset / 'samples.jsonl').write_text(json.dumps(dict(counts='counts.npy', stamp_ns=1000000000,
                                                                 steering=0.2, throttle=0.1))+'\n')
            for mode in ('steer_only', 'steer_throttle'):
                run = root / mode
                command = [sys.executable, '-m', 'kart_e2e.train', '--modality', 'evs',
                           '--train', str(dataset), '--validation', str(dataset),
                           '--repo', REPO, '--weights', str(weights), '--output', str(run),
                           '--mode', mode, '--fixed-throttle', '0.1', '--epochs', '1', '--num-workers', '0',
                           '--batch-size', '1', '--device', 'cpu']
                env = dict(os.environ, OMP_NUM_THREADS='2', MKL_NUM_THREADS='2')
                subprocess.run(command, check=True, env=env, capture_output=True, text=True)
                policy = load_checkpoint(run / 'best.pt', REPO)
                for key, value in policy.encoder.backbone.state_dict().items():
                    torch.testing.assert_close(value, encoder.state_dict()[key])
                counts = torch.zeros(1, 20, 120, 160)
                policy.train()
                policy(counts).sum().backward()
                self.assertTrue(all(v.grad is None for v in policy.encoder.parameters()))
                self.assertIsNotNone(policy.head[0].weight.grad)
                bundle = root / (mode+'-onnx')
                result = export(policy, bundle)
                self.assertLess(result['ort_max_abs_error'], 1e-5)
                self.assertEqual(model_contract(bundle)[1], mode)
                report = evaluate(bundle, dataset, root / (mode+'-eval'))
                self.assertEqual(report['samples'], 1)
            # Full finetune permits encoder gradients; 320x240 input has no padding.
            from kart_e2e.event_encoder_model import PortableEventEncoder
            from kart_e2e.evs_policy import EVSPolicy
            large = EVSPolicy(PortableEventEncoder(encoder, metadata(320)), 'steer_only', frozen=False)
            large(torch.zeros(1, 20, 240, 320)).sum().backward()
            self.assertIsNotNone(encoder.patch_embed.proj.weight.grad)
            result = export(large, root / 'large-onnx')
            self.assertEqual(result['input_shape'], [1, 20, 240, 320])
            self.assertLess(result['ort_max_abs_error'], 1e-5)

class EngineBindingTests(unittest.TestCase):
    def test_evs_engine_requires_exact_name_shape_and_fp32(self):
        from kart_e2e.engine_cache import validate_engine
        engine = types.SimpleNamespace(
            num_io_tensors=2,
            get_tensor_mode=lambda n: 'in' if n == 'input_tensor' else 'out',
            get_tensor_shape=lambda n: (1, 20, 240, 320) if n == 'input_tensor' else (1, 2),
            get_tensor_dtype=lambda n: 'fp32')
        trt = types.ModuleType('tensorrt')
        class Logger:
            WARNING = 1
            def __init__(self, *a): pass
        trt.Logger = Logger
        trt.Runtime = lambda logger: types.SimpleNamespace(deserialize_cuda_engine=lambda raw: engine)
        trt.TensorIOMode = types.SimpleNamespace(INPUT='in', OUTPUT='out')
        trt.float32 = 'fp32'
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, {'tensorrt': trt}):
            plan = Path(tmp) / 'model.plan'
            plan.write_bytes(b'fixture')
            validate_engine(plan, 'steer_throttle', 'input_tensor', [1, 20, 240, 320])
            with self.assertRaises(ValueError):
                validate_engine(plan, 'steer_throttle', 'input_tensor', [1, 20, 120, 160])
            with self.assertRaises(ValueError):
                validate_engine(plan, 'steer_only', 'input_tensor', [1, 20, 240, 320])
            engine.get_tensor_dtype = lambda name: 'fp16'
            with self.assertRaises(ValueError):
                validate_engine(plan, 'steer_throttle', 'input_tensor', [1, 20, 240, 320])
