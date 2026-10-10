"""Explicit VGL preparation from an existing official offline map."""

import os
import shutil
import tempfile
from pathlib import Path

from .storage import name, read_json


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
    size = [body.get(k) for k in ("width", "height")]
    if any(type(v) is not int or not 16 <= v <= 8192 for v in size):
        raise ValueError("画像サイズが不正です")
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
