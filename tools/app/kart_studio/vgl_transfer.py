"""Stage portable VGL maps paired by cuVSLAM content, never by folder name."""

import shutil

from .remote_agent import signature
from .storage import read_json, within


def copy_vgl_bundles(job, maps_root, source, destination):
    from kart_bringup.vgl_assets import (
        CONFIG_FILES,
        digest,
        extractor_shape,
        map_files,
        require_file,
    )

    root = maps_root / "vgl"
    if not root.exists() or not list((source / "cuvslam_map").glob("*.mdb")):
        job.log("VGL bundleなし: HDMap・VSLAM地図のみ送信します")
        return []
    root = within(maps_root, "vgl")
    expected = map_files(source)
    copied = []
    for candidate in sorted(root.iterdir()):
        if candidate.name.startswith(".") or not candidate.is_dir():
            continue
        candidate = within(root, candidate.name)
        profile_path = within(candidate, "vgl_profile.json", exists=False)
        if not profile_path.is_file():
            continue
        try:
            profile = read_json(profile_path)
        except ValueError:
            job.log(f"VGL profileを読めないため除外: {candidate.name}")
            continue
        if not isinstance(profile, dict) or profile.get("cuvslam_sha256") != expected:
            continue
        if (
            profile.get("schema") != "kart.vgl.v1"
            or profile.get("status") != "complete"
        ):
            job.log(f"未完成VGLを除外: {candidate.name}")
            continue
        job.check()
        before = signature(candidate)  # Reject links before copying.
        target = destination / "vgl" / candidate.name
        target.mkdir(parents=True)
        shutil.copyfile(profile_path, target / "vgl_profile.json")
        for part in ("cuvslam_map", "cuvgl_map", "vgl_runtime_config"):
            shutil.copytree(within(candidate, part), target / part)
        if signature(candidate) != before:
            raise ValueError(
                f"VGL bundleが転送準備中に変更されました: {candidate.name}"
            )
        # Validate the staged copy, including its actual map content.
        if map_files(target) != expected:
            raise ValueError(f"VGL bundleの元地図が一致しません: {candidate.name}")
        config = target / "vgl_runtime_config"
        for filename in CONFIG_FILES:
            if digest(require_file(config / filename)) != profile.get(
                "config_sha256", {}
            ).get(filename):
                raise ValueError(
                    f"VGL設定のハッシュ不一致: {candidate.name}/{filename}"
                )
        if extractor_shape((config / CONFIG_FILES[0]).read_text()) != profile.get(
            "input_shape"
        ):
            raise ValueError(f"VGL画像サイズが一致しません: {candidate.name}")
        for filename in ("keyframes/frames_meta.json", "bow_index.pb"):
            require_file(target / "cuvgl_map" / filename)
        if not any(
            p.is_file() and p.stat().st_size
            for p in (target / "cuvgl_map/vocabulary").rglob("*")
        ):
            raise ValueError(f"VGL vocabularyがありません: {candidate.name}")
        copied.append(candidate.name)
        job.log(f"VGL bundleを同梱: {candidate.name}")
    if not copied:
        job.log("対応する完成済みVGL bundleなし: HDMap・VSLAM地図のみ送信します")
    return copied
