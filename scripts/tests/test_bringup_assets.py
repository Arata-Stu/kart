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
