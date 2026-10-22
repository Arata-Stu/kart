"""Explicit VGL preparation from an existing official offline map."""

import os
import shutil
import tempfile
from pathlib import Path

from .storage import name, read_json


def model_option(model_dir):
    """Read exported shape from hash-matched metadata, never from a folder name."""
    from kart_bringup.vgl_assets import digest, model_files

    option = dict(path=str(model_dir), width=None, height=None, error="")
    try:
        root, onnx, _ = model_files(model_dir)
        option["path"] = str(root)
        option["onnx_sha256"] = digest(onnx)
        candidates = [root / "manifest.json"]
        if root.name == "runtime_models":
            candidates.append(root.parent / "manifest.json")
        manifest = next((p for p in candidates if p.is_file()), None)
        if manifest is not None:
            data = read_json(manifest)
            shape = data["input_shape"]
            if (not isinstance(shape, list) or len(shape) != 4 or shape[:2] != [1, 3]
                    or any(type(v) is not int or not 16 <= v <= 8192 for v in shape[2:])):
                raise ValueError("manifestの入力shapeが不正です")
            if option["onnx_sha256"] != data["onnx_sha256"]:
                raise ValueError("manifestとONNXのハッシュが一致しません")
            option.update(width=shape[3], height=shape[2])
    except (ValueError, KeyError, TypeError, OSError) as exc:
        option["error"] = str(exc)
    return option


def capture_option(source, bundle, models):
    """Find compatible local engines by content, independent of original PC paths."""
    from kart_bringup.vgl_assets import map_files, validate_bundle

    result = dict(path=str(bundle), input_shape=None, models=[], default_model="", error="")
    try:
        profile = read_json(bundle / "vgl_profile.json")
        shape = profile["input_shape"]
        if (profile.get("schema") != "kart.vgl.v1" or profile.get("status") != "complete"
                or not isinstance(shape, list) or len(shape) != 4 or shape[:2] != [1, 3]
                or any(type(v) is not int or v <= 0 for v in shape)):
            raise ValueError("VGL地図の入力shape・完成状態を確認できません")
        result["input_shape"] = shape
        if map_files(source) != profile["cuvslam_sha256"]:
            raise ValueError("VGL地図の元VSLAM地図が一致しません")
        for model in models:
            if model["error"] or model.get("onnx_sha256") != profile["onnx_sha256"]:
                continue
            if model["width"] is not None and [model["height"], model["width"]] != shape[2:]:
                continue
            try:
                validate_bundle(bundle, model["path"])
            except (ValueError, KeyError, TypeError, OSError):
                continue
            result["models"].append(model["path"])
        result["models"].sort(key=lambda p: (Path(p).name != "runtime_models", p))
        if result["models"]:
            result["default_model"] = result["models"][0]
        else:
            result["error"] = "対応する実行用モデルがありません。同じONNXのモデルをこのGPUでbuildしてください。"
    except (ValueError, KeyError, TypeError, OSError) as exc:
        result["error"] = str(exc)
    return result


def catalog(app, key):
    from kart_bringup.mission import discover
    from kart_bringup.prepare_vgl_map import source_frames

    source = app.maps.folder(key).resolve(strict=True)
    try:
        source_frames(source)
        error = ""
    except (ValueError, KeyError, OSError) as exc:
        error = "VGL用の公式保存画像/posesがありません: " + str(exc)
    models = sorted(
        set(discover(app.repo / "models", "models") + discover(app.maps.root, "models"))
    )
    defaults = read_json(
        app.repo / "ros2_ws/src/kart_bringup/config/localization/preparation.json"
    )
    root = app.maps.root / "vgl"
    index = 1
    while (root / f"{key}-vgl-v{index}").exists():
        index += 1
    bundles = sorted(p for p in root.glob(f"{key}-vgl-v*") if (p / "vgl_profile.json").is_file())
    options = [model_option(p) for p in models]
    return dict(
        models=[str(p) for p in models],
        model_options=options,
        defaults=defaults,
        error=error,
        name=f"{key}-vgl-v{index}",
        bundles=[str(p) for p in bundles],
        bundle_options=[capture_option(source, p, options) for p in bundles],
    )


def prepare(app, body):
    from kart_bringup.prepare_vgl_map import source_frames
    from kart_bringup.vgl_assets import model_files

    key = name(body.get("id"))
    source = app.maps.folder(key).resolve(strict=True)
    source_frames(source)
    models, _, _ = model_files(body.get("model_dir", ""))
    output = app.maps.root / "vgl" / name(body.get("name"))
    if output.exists():
        raise ValueError("同名のVGL bundleがあります。別名を指定してください")
    if not shutil.which("ros2"):
        raise ValueError("kart ROSコンテナで実行してください")
    option = model_option(models)
    if option["error"]:
        raise ValueError(option["error"])
    if option["width"] is not None:
        size = [option["width"], option["height"]]
        if any(k in body and (type(body[k]) is not int or body[k] != v)
               for k, v in zip(("width", "height"), size)):
            raise ValueError("画像サイズと選択モデルの入力解像度が一致しません")
    else:
        size = [body.get(k) for k in ("width", "height")]
        if any(type(v) is not int or not 16 <= v <= 8192 for v in size):
            raise ValueError("解像度不明モデルは画像幅・高さを手動指定してください")
    return key, source, models, output, size


def build(job, source, models, output, size):
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".vgl-ui-", dir=output.parent) as tmp:
        stage = Path(tmp) / "bundle"
        job.run(
            [
                "ros2",
                "run",
                "kart_bringup",
                "prepare_vgl_map",
                "--source-map",
                str(source),
                "--model-dir",
                str(models),
                "--output",
                str(stage),
                "--width",
                str(size[0]),
                "--height",
                str(size[1]),
            ]
        )
        profile = read_json(stage / "vgl_profile.json")
        if (
            profile.get("schema") != "kart.vgl.v1"
            or profile.get("status") != "complete"
        ):
            raise ValueError("完成したVGL bundleを確認できません")
        job.check()
        if output.exists():
            raise ValueError("同名のVGL bundleが作成されています")
        os.rename(stage, output)
    job.log(f"VGL bundle保存: {output}。bringupでVSLAM＋VGLを選択できます")
    return dict(bundle=str(output))
