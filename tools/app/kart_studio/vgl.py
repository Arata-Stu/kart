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
            if digest(onnx) != data["onnx_sha256"]:
                raise ValueError("manifestとONNXのハッシュが一致しません")
            option.update(width=shape[3], height=shape[2])
    except (ValueError, KeyError, TypeError, OSError) as exc:
        option["error"] = str(exc)
    return option


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
    return dict(
        models=[str(p) for p in models],
        model_options=[model_option(p) for p in models],
        defaults=defaults,
        error=error,
        name=f"{key}-vgl-v{index}",
        bundles=[
            str(p)
            for p in root.glob(f"{key}-vgl-v*")
            if (p / "vgl_profile.json").is_file()
        ],
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
