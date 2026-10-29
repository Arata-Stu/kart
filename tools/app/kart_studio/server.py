"""Small loopback HTTP adapter. All domain behavior lives in services."""

import argparse
import fcntl
import json
import mimetypes
import re
import shutil
import secrets
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit, urlencode

from . import capture, registration, vgl, dense, dense_steps, mesh_color
from .copies import copy_map
from .e2e import Learning
from .jobs import Jobs, Task
from .mapping import Mapping, local_records
from .maps import Maps
from .storage import atomic_json, name, read_json, within
from .transfers import Transfers, profile
from .tuning import Tuning, remote
from .audit import Audit, FILES as AUDIT_FILES


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
        self.jobs = Jobs(self.state / "jobs", repo, base)
        self.mapping = Mapping(repo, self.records, self.maps)
        self.transfers = Transfers(self.records, self.maps)
        self.learning = Learning(repo, base, self.records, self.state)
        self.tuning = Tuning(self.maps)
        self.audit = Audit(repo, base, self.records)
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
        if path == '/api/audits':
            result = self.audit.catalog()
            with self.jobs.lock:
                workers = list(self.jobs.jobs.values())
            result['running'] = [job.info() for job in workers if any(r.startswith('audit-output:') for r in job.resources) and job.status in ('running','orphaned')]
            return result
        if path == '/api/audit':
            return read_json(self.audit.artifact(key, 'summary.json'))
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
        if path == "/api/occupancy-overlay":
            return dense.overlay(self.maps, key)
        if path == "/api/mesh-overlay":
            return dense.mesh_overlay(self.maps, key)
        if path == "/api/dense-steps":
            return dense_steps.status(self.maps, key)
        if path == "/api/dense":
            return dense.catalog(self.maps, key)
        if path == "/api/vgl":
            return vgl.catalog(self, key)
        if path == "/api/e2e/loss":
            return self.learning.loss(key, q.get("kind", "runs"))
        if path == "/api/e2e/engine":
            folder = self.learning.folder("engines", key)
            return read_json(within(folder, "report.json"))
        if path == "/api/e2e/int8-benchmark":
            folder = self.learning.folder("models", q.get("id", ""))
            return read_json(within(folder, "benchmark/report.json"))
        if path == "/api/e2e/features":
            return read_json(within(self.learning.folder("features", key), "report.json"))
        if path == "/api/e2e/evaluation":
            folder = self.learning.folder("evaluations", key)
            return read_json(within(folder, "report.json"))
        if path == "/api/e2e/trash":
            from . import e2e_trash
            return e2e_trash.listing(self.learning)
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
        if path == '/api/audit/run':
            plan = self.audit.prepare(b)
            return self.jobs.start('走行記録の時刻解析: ' + plan['bag_key'],
                                   ['read:record:' + plan['bag_key'], 'audit-output:' + plan['key']],
                                   Task('audit', [plan]))
        if path in ("/api/esc/status", "/api/esc/start", "/api/esc/stop"):
            payload = {k: b[k] for k in ("run_id", "steps", "stream", "after", "generation") if k in b}
            return remote(b["connection"], "esc_" + path.rsplit("/", 1)[1], payload)
        if path == "/api/tuning/prepare":
            return self.tuning.prepare(b)
        if path in ("/api/tuning/status", "/api/tuning/apply", "/api/tuning/rollback", "/api/tuning/parameters_get", "/api/tuning/parameters_set", "/api/tuning/parking_start", "/api/tuning/parking_reset"):
            return self.tuning.request(b, path.rsplit("/", 1)[1])
        key = b.get("id", "")
        if path in ("/api/e2e/trash", "/api/e2e/restore"):
            from . import e2e_trash
            if path.endswith('/restore'):
                record = e2e_trash.entry(self.learning, b.get('token'))
                kind, key = record['kind'], record['id']
            else:
                kind, key = b.get('kind'), b.get('id')
                self.learning.folder(kind, key)
            with self.jobs.guard('e2e-trash', 'e2e:' + kind + ':' + key):
                if path.endswith('/restore'):
                    return e2e_trash.restore(self.learning, b['token'])
                return e2e_trash.move(self.learning, kind, key)
        if path == "/api/e2e/settings":
            return self.learning.save_settings(b)
        if path in (
            "/api/e2e/section-routing",
            "/api/e2e/section-offline",
            "/api/e2e/section-dataset",
            "/api/e2e/dataset",
            "/api/e2e/train",
            "/api/e2e/distill",
            "/api/e2e/export",
            "/api/e2e/evaluate",
            "/api/e2e/build-engine",
            "/api/e2e/quantize",
            "/api/e2e/features",
        ):
            action = path.rsplit("/", 1)[1]
            plan = self.learning.prepare(action, b)
            titles = {
                "section-routing": "section → head割当の保存",
                "section-offline": "offline VSLAM・pose取得",
                "section-dataset": "section dataset作成",
                "features": "特徴量の可視化",
                "dataset": "データセット作成",
                "train": "制御モデル学習",
                "distill": "ViT-S → ViT-Tiny特徴蒸留",
                "export": "ONNX export",
                "evaluate": "rosbagオフライン評価",
                "build-engine": "Notebook TensorRT build",
                "quantize": "INT8校正＋FP32比較benchmark",
            }
            return self.jobs.start(
                titles[action] + ": " + plan["key"],
                plan["resources"] + ["e2e:" + plan["kind"] + ":" + plan["key"]],
                Task('learning', [plan]),
            )
        if path == "/api/e2e/push":
            p = profile(b["connection"])
            key = b["id"]
            from .transfers import remote_path

            root = remote_path(b["model_root"])
            self.learning.folder("models", key)
            return self.jobs.start(
                "モデル送信: " + key,
                ["read:e2e:models:" + key],
                Task('model_push', [p, key, root]), queue="scp",
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
            relatives = b.get("relatives")
            multiple = relatives is not None
            if not multiple:
                relatives = [b["relative"]]
            if not isinstance(relatives, list) or not 1 <= len(relatives) <= 200:
                raise ValueError("bagを1〜200件選択してください")
            tasks = []
            for relative in relatives:
                if not isinstance(relative, str):
                    raise ValueError("bagの相対パスを指定してください")
                key, _ = self.transfers.pull_target(relative, None if multiple else b.get("name"))
                tasks.append(("bagを受信: " + key, ["record:" + key],
                              Task('record_pull', [p, relative, None if multiple else b.get("name")])))
            jobs = self.jobs.start_many(tasks, queue="scp")
            return dict(jobs=jobs) if multiple else jobs[0]
        if path == "/api/push":
            p = profile(b["connection"])
            name(key)
            return self.jobs.start(
                "地図を送信: " + key,
                ["read:map:" + key],
                Task('map_push', [p, key]), queue="scp",
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
                ["read:map:" + source, "map:" + destination],
                Task('map_copy', [source, destination, b['revision']]),
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
                Task('line_generate', [key, b['kind'], b.get('settings', {}), b['revision'], b.get('lane_id')]),
            )
        if path == "/api/parking-preview":
            from .parking_preview import preview
            document=self.maps.load(key)
            if document['revision']!=b['revision']: raise ValueError('地図が変わりました。試走を開き直してください')
            return preview(document,b['lane_id'],b['line_type'],b['slot'],b['pose'],b.get('options'),b.get('initial_speed',0.))
        if path == "/api/tracking-preview":
            from .tracking_preview import preview
            return preview(b)
        if path == "/api/parking-validate":
            from .parking import validate
            return validate(b["document"])
        if path == "/api/export":
            from .parking import warnings as parking_warnings
            with self.jobs.guard("map:" + name(key)):
                output = self.maps.export(key)
                return dict(
                    warnings=parking_warnings(self.maps.load(key)),
                    path=str(output), files=sorted(p.name for p in output.iterdir())
                )
        if path == "/api/dense":
            return dense.catalog(self.maps, key)
        if path == "/api/vgl":
            key, source, models, output, size = vgl.prepare(self, b)
            return self.jobs.start(
                "VGL地図生成: " + key,
                ["read:map:" + key, "vgl:" + str(output)],
                Task('vgl', [source, models, output, size]),
            )
        if path == "/api/mesh-color":
            name(key)
            inputs = mesh_color.validate(self, key, b)
            return self.jobs.start(
                "mesh RGB着色: " + key, ["map:" + key],
                Task('mesh_color', [key, b, inputs]),
            )
        if path == "/api/dense-step":
            name(key)
            step, resolution = dense_steps.validate(self, key, b)
            return self.jobs.start(
                ("深度推定: " if step == "depth" else "占有地図・メッシュ生成: ") + key,
                ["map:" + key, "ros:" + str(self.mapping.defaults['ros_domain_id'])],
                Task('dense_step', [key, b['revision'], step, resolution]),
            )
        if path == "/api/capture":
            name(key)
            bag, workflow, options = capture.validate(self, key, b)
            return self.jobs.start(
                "HDMap用点群取得: " + key,
                ["map:" + key, "ros:" + str(workflow["ros_domain_id"])],
                Task('capture', [key, b['revision'], bag, workflow, options]),
            )
        if path == "/api/build":
            key, bag, workflow = self.mapping.validate(b)
            return self.jobs.start(
                "VSLAM地図作成: " + key,
                ["map:" + key, "ros:" + str(workflow["ros_domain_id"])],
                Task('map_build', [key, bag, workflow]),
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

    def trusted(self, report_token=None):
        host = self.headers.get("Host", "")
        expected = {
            f"127.0.0.1:{self.server.server_port}",
            f"localhost:{self.server.server_port}",
        }
        if host not in expected:
            raise ValueError("Invalid Host")
        if report_token and secrets.compare_digest(report_token, self.server.studio.audit.download_token):
            return  # Read-only artifact links from our opaque-origin sandbox; POST never uses this.
        origin = self.headers.get("Origin")
        if origin and origin != f"http://{host}":
            raise ValueError("Cross-origin request refused")
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            raise ValueError("Cross-site request refused")

    def do_GET(self):
        try:
            url = urlsplit(self.path)
            query = {k: v[-1] for k, v in parse_qs(url.query).items()}
            self.trusted(query.get('token') if url.path == '/audit-file' else None)
            if url.path == '/audit-file':
                query = {k: v[-1] for k, v in parse_qs(url.query).items()}
                filename = query.get('file', 'report.html')
                path = self.server.studio.audit.artifact(query.get('id', ''), filename)
                if filename == 'report.html':
                    nonce = secrets.token_urlsafe(24)
                    body = path.read_text()
                    body = body.replace('<h2>時系列</h2>', '<h2 id="timeseries">時系列</h2>')
                    for artifact in AUDIT_FILES:
                        link = '/audit-file?' + urlencode(dict(id=query['id'], file=artifact, token=self.server.studio.audit.download_token))
                        body = body.replace('href="' + artifact + '"', 'href="' + link.replace('&', '&amp;') + '"')
                    body = body.replace('<script>', '<script nonce="' + nonce + '">')
                    raw = body.encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(raw)))
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Referrer-Policy', 'no-referrer')
                    self.send_header('X-Content-Type-Options', 'nosniff')
                    self.send_header('Content-Security-Policy',
                        "default-src 'none'; script-src 'nonce-" + nonce + "'; style-src 'unsafe-inline'; "
                        "img-src data: blob:; frame-ancestors 'self'; sandbox allow-scripts allow-downloads")
                    self.end_headers()
                    self.wfile.write(raw)
                else:
                    with path.open('rb') as stream:
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/octet-stream')
                        self.send_header('Content-Length', str(path.stat().st_size))
                        self.send_header('Content-Disposition', 'attachment; filename="' + filename + '"')
                        self.send_header('X-Content-Type-Options', 'nosniff')
                        self.end_headers()
                        shutil.copyfileobj(stream, self.wfile, length=1024 * 1024)
                return
            if url.path == "/e2e-feature-image":
                query = {k: v[-1] for k, v in parse_qs(url.query).items()}
                filename = query.get("file", "")
                if not re.fullmatch(r"frames/[0-9]{4}/(?:rgb|pca|saliency|channels)\.png", filename):
                    raise ValueError("Invalid feature image")
                folder = self.server.studio.learning.folder("features", query.get("id", ""))
                return self.reply(200, within(folder, filename).read_bytes(), "image/png")
            if url.path == "/dense-file":
                query = {k: v[-1] for k, v in parse_qs(url.query).items()}
                path = dense.artifact(self.server.studio.maps, query.get("id", ""), query.get("file", ""))
                with path.open("rb") as stream:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/png" if path.suffix == ".png" else "application/octet-stream")
                    self.send_header("Content-Length", str(path.stat().st_size))
                    self.send_header("X-Content-Type-Options", "nosniff")
                    if path.suffix != ".png":
                        self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
                    self.end_headers()
                    shutil.copyfileobj(stream, self.wfile, length=1024 * 1024)
                return
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
