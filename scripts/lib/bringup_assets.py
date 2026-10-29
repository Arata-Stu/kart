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


def select_map_directory(hdmaps, root, choose):
    """Select a transfer directory once; ask about HDMaps only if ambiguous."""
    root = Path(root).expanduser().resolve()
    groups = {}
    for hdmap in hdmaps:
        path = Path(hdmap).expanduser().resolve()
        directory = path.parent
        for parent in path.parents:
            if not parent.is_relative_to(root):
                break
            if (parent / 'cuvslam_map').is_dir():
                directory = parent
                break
        groups.setdefault(directory, []).append(hdmap)
    directory = choose('地図フォルダー', sorted(groups))
    return select_related('フォルダー内のHDMap', groups[directory], groups[directory], choose)


def matching_resolution(candidates, infra_profile):
    """Only narrow already paired bundles; resolution alone is not identity."""
    width, height, _ = infra_profile.split('x')
    expected = [1, 3, int(height), int(width)]
    result = []
    for candidate in candidates:
        try:
            profile = json.loads((Path(candidate) / 'vgl_profile.json').read_text())
            if profile.get('input_shape') == expected:
                result.append(candidate)
        except (OSError, ValueError, AttributeError):
            pass
    return result


def select_runtime_model(bundle, candidates, choose):
    matched = paired_models(bundle, candidates)
    # A source engine may have been built on another GPU. Prefer the runtime
    # export, but retain source-only installations and explicit CLI overrides.
    ordered = sorted(matched or candidates,
                     key=lambda p: (Path(p).name != 'runtime_models', str(p)))
    runtime = [p for p in matched if Path(p).name == 'runtime_models']
    if len(runtime) == 1:
        return select_related('VGL runtimeモデル', ordered, runtime, choose)
    return select_related('VGLモデル（実行GPU用engine）', ordered,
                          ordered if matched else [], choose)
