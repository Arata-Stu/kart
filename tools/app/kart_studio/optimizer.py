"""Optional helper optimizer in a separate, cancellable Python environment."""

import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def optimize(document, parameters, check):
    if not document["closed"]:
        raise ValueError(
            "helper最適化は閉路に対応しています。開路は簡易方式を選択してください"
        )
    python = os.environ.get("KART_RACELINE_PYTHON") or (
        "/opt/env/bin/python"
        if Path("/opt/env/bin/python").is_file()
        else sys.executable
    )
    with tempfile.TemporaryDirectory(prefix="kart-raceline-") as tmp:
        root = Path(tmp)
        source, output = root / "input.json", root / "output.json"
        source.write_text(json.dumps(dict(document=document, parameters=parameters)))
        env = dict(
            os.environ,
            PYTHONPATH=str(Path(__file__).resolve().parent.parent),
            MPLCONFIGDIR=str(root / "mpl"),
            MPLBACKEND="Agg",
        )
        with (root / "log.txt").open("w") as log:
            process = subprocess.Popen(
                [
                    python,
                    "-m",
                    "kart_studio.optimizer_worker",
                    str(source),
                    str(output),
                ],
                stdout=log,
                stderr=log,
                env=env,
            )
            try:
                deadline = time.monotonic() + 600
                while process.poll() is None:
                    check()
                    if time.monotonic() > deadline:
                        raise ValueError("最適化が600秒以内に完了しませんでした")
                    time.sleep(0.1)
                check()
                if process.returncode:
                    detail = (root / "log.txt").read_text()[-2400:]
                    raise ValueError(
                        "helper最適化に失敗しました。KART_RACELINE_PYTHONと依存を確認してください。\n"
                        + detail
                    )
                return json.loads(output.read_text())
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
