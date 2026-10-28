"""Evidence-based candidate narrowing; final localization validation remains mandatory."""
import json
from pathlib import Path


def paired_maps(hdmap, candidates, root):
    from kart_bringup.vgl_assets import map_files
    path=Path(hdmap).resolve().parent
    root=Path(root).expanduser().resolve()
    if not path.is_relative_to(root): return []
    expected=None
    for parent in (path,*path.parents):
        if not parent.is_relative_to(root): break
        if (parent/'cuvslam_map').is_dir():
            expected=map_files(parent);break
    if expected is None: return []
    result=[]
    for candidate in candidates:
        try:
            profile=Path(candidate)/'vgl_profile.json'
            identity=json.loads(profile.read_text()).get('cuvslam_sha256') if profile.is_file() else map_files(Path(candidate))
            if identity==expected: result.append(candidate)
        except (OSError,ValueError,TypeError,AttributeError): pass
    return result


def paired_models(bundle, candidates):
    from kart_bringup.vgl_assets import model_files,digest
    try: expected=json.loads((Path(bundle)/'vgl_profile.json').read_text())['onnx_sha256']
    except (OSError,ValueError,KeyError): return []
    result=[]
    for candidate in candidates:
        try:
            _,onnx,_=model_files(candidate)
            if digest(onnx)==expected: result.append(candidate)
        except (OSError,ValueError,TypeError,AttributeError): pass
    return result


def select_related(title, candidates, matched, choose):
    if len(matched)==1:
        print(f'{title}: {matched[0]}（対応する候補を自動選択）')
        return matched[0]
    return choose(title, matched or candidates)
