"""Validate UI job requests and run the installed offline ROS worker."""

import copy
import os
import re
import shutil
import uuid
from pathlib import Path

from .jobs import Cancelled
from .geometry import number
from .storage import atomic_json, name, read_json, within


class Mapping:
    def __init__(self, repo, records, maps):
        self.repo, self.records, self.maps = repo, records, maps
        self.defaults = read_json(
            repo / "ros2_ws/src/kart_bringup/config/mapping/workflow.json"
        )

    def environment(self):
        return dict(
            ros=bool(shutil.which("ros2")),
            defaults=self.defaults,
            message="CUDA対応Linuxのkart ROS環境で実行します。Macでは点群の編集・転送を利用できます。",
        )

    def validate(self, body):
        key = name(body.get("name"))
        bag = within(self.records, body.get("bag", ""))
        if not (bag / "metadata.yaml").is_file():
            raise ValueError("metadata.yamlがあるbagを選択してください")
        if (self.maps.root / key).exists():
            raise ValueError("同名の地図があります")
        if not shutil.which("ros2"):
            raise ValueError(
                "ros2がありません。CUDA対応Linuxのkartコンテナ内でUIを起動してください"
            )
        workflow = copy.deepcopy(self.defaults)
        raw = body.get("workflow", {})
        unknown = set(raw) - set(workflow)
        if unknown:
            raise ValueError("未知の設定: " + ", ".join(unknown))
        workflow.update(raw)
        for k in ("left_image", "right_image", "left_info", "right_info"):
            if not re.fullmatch(r"/[A-Za-z0-9_/]+", workflow[k]):
                raise ValueError("topic名が不正です")
        if workflow["tf_static"] != "/tf_static":
            raise ValueError("tf_staticは/tf_static固定です")
        workflow["ros_domain_id"] = int(
            number(workflow["ros_domain_id"], 1, 232, "ROS domain")
        )
        overrides = workflow["overrides"]
        if not isinstance(overrides, dict) or set(overrides) - {
            "tracking_mode",
            "base_frame",
            "camera_optical_frames",
        }:
            raise ValueError("VSLAM overrideが不正です")
        workflow["overrides"] = overrides = dict(
            self.defaults["overrides"], **overrides
        )
        if overrides.get("tracking_mode", 0) != 0:
            raise ValueError("公式offline工程はStereoのみ対応しています")
        frames = [overrides[k] for k in ("base_frame",) if k in overrides]
        if "camera_optical_frames" in overrides:
            optical = overrides["camera_optical_frames"]
            if not isinstance(optical, list) or len(optical) != 2:
                raise ValueError("左右2つの光学frameが必要です")
            frames += optical
        if any(
            not isinstance(f, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]*", f)
            for f in frames
        ):
            raise ValueError("frame名が不正です")
        return key, bag, workflow

    def build(self, job, key, bag, workflow):
        stage = self.maps.root / (".build-" + uuid.uuid4().hex)
        stage.mkdir()
        job_file = stage / "job.json"
        atomic_json(job_file, dict(bag=str(bag), output=str(stage), workflow=workflow))
        env = dict(
            os.environ,
            ISAAC_ROS_WS=os.environ.get("ISAAC_ROS_WS", str(self.repo / "ros2_ws")),
            ROS_DOMAIN_ID=str(workflow["ros_domain_id"]),
            ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
        )
        job.log(f"隔離ROS domain {workflow['ros_domain_id']} / 一時成果物 {stage}")
        try:
            job.run(
                ["ros2", "run", "kart_mapping", "build_map", "--job", str(job_file)],
                env=env,
            )
            self.maps.initialize_pending(stage, key)
            job.check()
            if (self.maps.root / key).exists():
                raise ValueError("同名の地図があります")
            os.rename(stage, self.maps.root / key)
            return dict(map=key)
        except Cancelled:
            raise
        except Exception:
            if stage.exists():
                failed = self.maps.root / ".failed"
                failed.mkdir(exist_ok=True)
                retained = failed / stage.name.removeprefix(".build-")
                os.rename(stage, retained)
                job.log(f"失敗時の診断データを保存しました: {retained}")
            raise
        finally:
            if stage.exists():
                shutil.rmtree(stage)


def local_records(root):
    result = []
    for current, dirs, files in os.walk(root, followlinks=False):
        path = Path(current)
        dirs[:] = sorted(
            d for d in dirs if not d.startswith(".") and not (path / d).is_symlink()
        )
        if "metadata.yaml" in files:
            if not (path / "metadata.yaml").is_symlink():
                result.append(
                    dict(
                        id=str(path.relative_to(root)),
                        name=path.name,
                        modified=path.stat().st_mtime,
                    )
                )
            dirs[:] = []
        if len(result) >= 1000:
            break
    return sorted(result, key=lambda d: d["modified"], reverse=True)
