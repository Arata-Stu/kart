"""Small loopback HTTP adapter. All domain behavior lives in services."""

import argparse
import fcntl
import json
import mimetypes
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from . import capture, registration
from .copies import copy_map
from .e2e import Learning
from .jobs import Jobs
from .mapping import Mapping, local_records
from .maps import Maps
from .storage import atomic_json, name, read_json, within
from .transfers import Transfers, profile


class Studio:
    def __init__(self, repo, data=None):
        self.repo = repo
        self.frontend = repo / "tools/app/frontend"
        base = data or repo
        self.state = base / ".kart-studio"
        self.records = base / "record"
        self.maps = Maps(base / "map")
        self.state.mkdir(parents=True, exist_ok=True)
        self.instance_lock = (self.state / "server.lock").open("a")
        try:
            fcntl.flock(self.instance_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError(
                "同じデータのStudioが起動済みです。別データには--data-rootを使ってください"
            )
        self.jobs = Jobs(self.state / "jobs")
        self.mapping = Mapping(repo, self.records, self.maps)
        self.transfers = Transfers(self.records, self.maps)
        self.learning = Learning(repo, base, self.records, self.state)
        self.profile_file = self.state / "connection.json"

    def connection_defaults(self):
        hosts = read_json(self.repo / "config/jetson_hosts.json")
        return dict(
            user="kart",
            host=next(
                p["host"] for p in hosts["presets"] if p["id"] == hosts["default"]
            ),
            port=22,
            record_root="/home/kart/workspaces/kart/record",
            map_root="/home/kart/workspaces/kart/map",
        )

    def get(self, path, q):
        key = q.get("id", "")
        if path == "/api/config":
            hosts = read_json(self.repo / "config/jetson_hosts.json")
            connection = (
                read_json(self.profile_file)
                if self.profile_file.exists()
                else self.connection_defaults()
            )
            return dict(
                connection=connection,
                jetson_hosts=hosts,
                environment=self.mapping.environment(),
                records=str(self.records),
                maps=str(self.maps.root),
            )
        if path == "/api/e2e/evaluation":
            folder = self.learning.folder("evaluations", key)
            return read_json(within(folder, "report.json"))
        if path == "/api/e2e":
            return self.learning.catalog()
        if path == "/api/trash":
            return self.maps.trashed()
        if path == "/api/maps":
            return self.maps.list()
        if path == "/api/map":
            return self.maps.load(key)
        if path == "/api/cloud":
            if self.maps.load(key).get("snapshot_status") == "pending":
                return dict(
                    points=[],
                    source_count=0,
                    trajectory=[],
                    provenance=dict(
                        warning="VSLAM地図は作成済み。HDMap用点群は未取得です。CUDA対応Linuxで「保存地図から点群を取得」を実行してください。"
                    ),
                )
            return read_json(within(self.maps.folder(key), "cloud.json"))
        if path == "/api/records":
            return local_records(self.records)
        if path == "/api/jobs":
            return self.jobs.list()
        if path == "/api/log":
            if not re.fullmatch(r"[a-f0-9]{12}", key):
                raise ValueError("Invalid job id")
            log = within(self.state / "jobs", key + ".log")
            with log.open("rb") as stream:
                stream.seek(max(0, log.stat().st_size - 64000))
                return dict(text=stream.read().decode(errors="replace"))
        raise KeyError("Not found")

    def post(self, path, b):
        key = b.get("id", "")
        if path == "/api/e2e/settings":
            return self.learning.save_settings(b)
        if path in (
            "/api/e2e/dataset",
            "/api/e2e/train",
            "/api/e2e/export",
            "/api/e2e/evaluate",
        ):
            action = path.rsplit("/", 1)[1]
            plan = self.learning.prepare(action, b)
            titles = {
                "dataset": "データセット作成",
                "train": "DINOv3学習",
                "export": "ONNX export",
                "evaluate": "rosbagオフライン評価",
            }
            return self.jobs.start(
                titles[action] + ": " + plan["key"],
                plan["resources"] + ["e2e:" + plan["kind"] + ":" + plan["key"]],
                lambda job: self.learning.execute(job, plan),
            )
        if path == "/api/e2e/push":
            p = profile(b["connection"])
            key = name(b["id"])
            from .transfers import remote_path

            root = remote_path(b["model_root"])
            self.learning.folder("models", key)
            return self.jobs.start(
                "モデル送信: " + key,
                ["e2e:models:" + key],
                lambda job: self.learning.push(job, p, key, root),
            )
        if path == "/api/connection/reset":
            value = self.connection_defaults()
            atomic_json(self.profile_file, value)
            return value
        if path == "/api/connection":
            value = profile(b)
            atomic_json(self.profile_file, value)
            return value
        if path == "/api/browse":
            return self.transfers.browse(b["connection"], b.get("relative", ""))
        if path == "/api/pull":
            p = profile(b["connection"])
            key = name(b["name"])
            return self.jobs.start(
                "bagを受信: " + key,
                ["record:" + key],
                lambda job: self.transfers.pull(job, p, b["relative"], key),
            )
        if path == "/api/push":
            p = profile(b["connection"])
            name(key)
            return self.jobs.start(
                "地図を送信: " + key,
                ["map:" + key],
                lambda job: self.transfers.push(job, p, key),
            )
        if path == "/api/import":
            key = name(b["name"])
            with self.jobs.guard("map:" + key):
                return self.maps.import_snapshot(key, Path(b["path"]))
        if path in ("/api/registration-preview", "/api/registration-apply"):
            with self.jobs.guard("map:" + name(key), "map:" + name(b["source"])):
                self.maps.require_snapshot(key)
                self.maps.require_snapshot(b["source"])
                fn = (
                    registration.apply
                    if path.endswith("-apply")
                    else registration.preview
                )
                return fn(self.maps, key, b)
        if path == "/api/copy-map":
            source, destination = name(key), name(b["name"])
            return self.jobs.start(
                "地図を複製: " + source,
                ["map:" + source, "map:" + destination],
                lambda job: copy_map(
                    self.maps, job, source, destination, b["revision"]
                ),
            )
        if path == "/api/delete-map":
            with self.jobs.guard("map:" + name(key), "map-trash"):
                return self.maps.trash(key, b["revision"])
        if path == "/api/restore-map":
            token = b["token"]
            if not isinstance(token, str) or "--" not in token:
                raise ValueError("Invalid trash token")
            original = name(token.rsplit("--", 1)[0])
            with self.jobs.guard("map:" + original, "map-trash"):
                return self.maps.restore(token)
        if path == "/api/save":
            with self.jobs.guard("map:" + name(key)):
                return self.maps.save(key, b["document"])
        if path == "/api/generate":
            if b["kind"] not in ("centerline", "raceline", "customline"):
                raise ValueError("Invalid line kind")
            return self.jobs.start(
                b["kind"] + "生成",
                ["map:" + name(key)],
                lambda job: self.maps.generate(
                    key,
                    b["kind"],
                    b.get("settings", {}),
                    b["revision"],
                    job.check,
                    b.get("lane_id"),
                ),
            )
        if path == "/api/export":
            with self.jobs.guard("map:" + name(key)):
                output = self.maps.export(key)
                return dict(
                    path=str(output), files=sorted(p.name for p in output.iterdir())
                )
        if path == "/api/capture":
            name(key)
            bag, workflow, options = capture.validate(self, key, b)
            return self.jobs.start(
                "HDMap用点群取得: " + key,
                ["map:" + key, "ros:" + str(workflow["ros_domain_id"])],
                lambda job: capture.capture(
                    self, job, key, b["revision"], bag, workflow, options
                ),
            )
        if path == "/api/build":
            key, bag, workflow = self.mapping.validate(b)
            return self.jobs.start(
                "VSLAM地図作成: " + key,
                ["map:" + key, "ros:" + str(workflow["ros_domain_id"])],
                lambda job: self.mapping.build(job, key, bag, workflow),
            )
        if path == "/api/stop":
            job = self.jobs.jobs.get(key)
            if not job:
                raise ValueError("実行中ジョブがありません")
            job.stop()
            return job.info()
        raise KeyError("Not found")


class Handler(BaseHTTPRequestHandler):
    def log_request(self, code="-", size="-"):
        if self.command != "GET" or code != 200:
            super().log_request(code, size)

    def reply(self, code, body, kind="application/json; charset=utf-8"):
        raw = (
            body
            if isinstance(body, bytes)
            else json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
        )
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; frame-ancestors 'none'",
        )
        self.end_headers()
        self.wfile.write(raw)

    def trusted(self):
        host = self.headers.get("Host", "")
        expected = {
            f"127.0.0.1:{self.server.server_port}",
            f"localhost:{self.server.server_port}",
        }
        if host not in expected:
            raise ValueError("Invalid Host")
        origin = self.headers.get("Origin")
        if origin and origin != f"http://{host}":
            raise ValueError("Cross-origin request refused")
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            raise ValueError("Cross-site request refused")

    def do_GET(self):
        try:
            self.trusted()
            url = urlsplit(self.path)
            if url.path.startswith("/api/"):
                query = {k: v[-1] for k, v in parse_qs(url.query).items()}
                return self.reply(200, self.server.studio.get(url.path, query))
            filename = "index.html" if url.path == "/" else url.path.lstrip("/")
            path = within(self.server.studio.frontend, filename)
            if path.suffix not in (".html", ".css", ".js", ".svg"):
                raise KeyError("Not found")
            self.reply(
                200,
                path.read_bytes(),
                mimetypes.guess_type(path.name)[0] or "text/plain",
            )
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.reply(400, dict(error=str(exc)))

    def do_POST(self):
        try:
            self.trusted()
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise ValueError("JSON required")
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4 * 1024 * 1024:
                raise ValueError("Body size limit exceeded")
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("JSON object required")
            self.reply(200, self.server.studio.post(urlsplit(self.path).path, body))
        except (ValueError, KeyError, TypeError, OSError, TimeoutError) as exc:
            self.reply(400, dict(error=str(exc)))
        except Exception as exc:
            self.log_error("%s", exc)
            self.reply(
                500, dict(error="処理に失敗しました。サーバーログを確認してください")
            )


def main():
    parser = argparse.ArgumentParser(description="Kart Map Studio (loopback only)")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument(
        "--data-root", type=Path, help="Optional isolated data folder, e.g. for a demo"
    )
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.studio = Studio(repo, args.data_root.resolve() if args.data_root else None)
    print(f"Kart Map Studio: http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.studio.jobs.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
