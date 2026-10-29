import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock, patch

spec=importlib.util.spec_from_file_location('bringup_assets',Path(__file__).parents[1]/'lib/bringup_assets.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)


def test_unique_matching_selection_and_ambiguity():
    chooser=Mock(return_value='chosen')
    assert a.select_related('map',['x','y'],['y'],chooser)=='y'
    chooser.assert_not_called()
    assert a.select_related('map',['x','y'],[],chooser)=='chosen'
    chooser.assert_called_with('map',['x','y'])
    a.select_related('map',['x','y','z'],['x','y'],chooser)
    chooser.assert_called_with('map',['x','y'])


def test_content_pairing_not_folder_names(tmp_path):
    for name,data in [('map',b'abc'),('same',b'abc'),('other',b'def')]:
        p=tmp_path/name/'cuvslam_map';p.mkdir(parents=True);(p/'map.mdb').write_bytes(data)
    export=tmp_path/'map/export';export.mkdir();hd=export/'hd_map.yaml';hd.write_text('{}')
    assert a.paired_maps(hd,[tmp_path/'same',tmp_path/'other'],tmp_path)==[tmp_path/'same']
    bundle=tmp_path/'bundle';bundle.mkdir();(bundle/'vgl_profile.json').write_text(json.dumps(dict(onnx_sha256='match')))
    with patch('kart_bringup.vgl_assets.model_files',side_effect=lambda p:(p,p,[])), patch('kart_bringup.vgl_assets.digest',side_effect=lambda p:'match' if p=='good' else 'bad'):
        assert a.paired_models(bundle,['good','bad'])==['good']


def test_directory_selection_groups_exports_and_preserves_ambiguity(tmp_path):
    root = tmp_path / 'course'
    (root / 'cuvslam_map').mkdir(parents=True)
    export = root / 'export'
    export.mkdir()
    hd = export / 'hd_map.yaml'
    hd.write_text('{}')
    chooser = Mock(return_value=root)
    assert a.select_map_directory([hd], tmp_path, chooser) == hd
    chooser.assert_called_once_with('地図フォルダー', [root])
    second = root / 'map.json'
    chooser = Mock(side_effect=[root, second])
    assert a.select_map_directory([hd, second], tmp_path, chooser) == second
    assert chooser.call_count == 2


def test_bundle_resolution_and_runtime_preference(tmp_path):
    bundles = []
    for size in [(240, 424), (480, 848)]:
        bundle = tmp_path / str(size[1])
        bundle.mkdir()
        (bundle / 'vgl_profile.json').write_text(json.dumps({'input_shape': [1, 3, *size]}))
        bundles.append(bundle)
    assert a.matching_resolution(bundles, '848x480x60') == [bundles[1]]
    assert a.matching_resolution(bundles, '640x480x30') == []
    source = tmp_path / 'source_models'
    runtime = tmp_path / 'runtime_models'
    unrelated = tmp_path / 'other/runtime_models'
    chooser = Mock()
    with patch.object(a, 'paired_models', return_value=[source, runtime]):
        assert a.select_runtime_model(bundles[0], [source, unrelated, runtime], chooser) == runtime
        chooser.assert_not_called()
    with patch.object(a, 'paired_models', return_value=[]):
        a.select_runtime_model(bundles[0], [source, runtime], chooser)
        chooser.assert_called_with('VGLモデル（実行GPU用engine）', [runtime, source])
    with patch.object(a, 'paired_models', return_value=[source]):
        assert a.select_runtime_model(bundles[0], [source, unrelated], chooser) == source


def test_drive_one_map_prompt_selects_matching_bundle_and_runtime(tmp_path):
    import contextlib
    import io
    import sys
    spec = importlib.util.spec_from_file_location('drive_bringup', Path(__file__).parents[1] / 'lib/bringup.py')
    ui = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ui)
    course = tmp_path / 'course'
    (course / 'cuvslam_map').mkdir(parents=True)
    (course / 'cuvslam_map/map.mdb').write_bytes(b'map')
    hdmap = course / 'map.json'
    hdmap.write_text('{}')
    from kart_bringup.vgl_assets import map_files
    bundles = []
    for width, height in [(424, 240), (640, 480)]:
        bundle = course / f'vgl_{width}'
        bundle.mkdir()
        (bundle / 'vgl_profile.json').write_text(json.dumps({
            'cuvslam_sha256': map_files(course), 'input_shape': [1, 3, height, width]}))
        bundles.append(bundle)
    runtime = tmp_path / 'runtime_models'
    source = tmp_path / 'source_models'
    prompts = []
    def choose(title, values, label=str, default=0):
        prompts.append(title)
        if title == '用途': return 'drive'
        if title == '地図フォルダー': return course
        return values[default]
    def discover(root, kind):
        return {'hdmap': [hdmap], 'bundle': bundles, 'models': [source, runtime]}.get(kind, [])
    argv = ['bringup.py', '--dry-run', '--no-bridge', '--no-evs', '--no-tuning',
            '--map-root', str(tmp_path), '--rgb-profile', '640x480x30',
            '--infra-profile', '640x480x60', '--imu-profile', 'high',
            '--auto-fixed-throttle=-1', '--run-name', 'selection_test']
    output = io.StringIO()
    with (patch.object(sys, 'argv', argv), patch.object(sys.stdin, 'isatty', return_value=True),
          patch.object(ui, 'choose', side_effect=choose),
          patch('kart_bringup.mission.discover', side_effect=discover),
          patch('kart_bringup.mission.hdmap_choices', return_value=[('lane', 'centerline')]),
          patch('bringup_assets.paired_models', return_value=[source, runtime]),
          patch('kart_bringup.localization.resolve', return_value=(
              {'cuvslam': {'tracking_mode': 0}}, {}, {}, {'input_shape': [1, 3, 480, 640]})),
          contextlib.redirect_stdout(output)):
        ui.main()
    assert prompts.count('地図フォルダー') == 1
    assert not any('HDMap' in title or 'bundle' in title or 'モデル' in title for title in prompts)
    assert f'map_file:={hdmap}' in output.getvalue()
    assert f'map_dir:={bundles[1]}' in output.getvalue()
    assert f'model_dir:={runtime}' in output.getvalue()
