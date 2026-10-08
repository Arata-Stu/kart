"""ROS input bridge with a localhost browser editor and terminal wizard."""
import argparse
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time

import yaml

from . import profile


class Bridge:
    def __init__(self, namespace, ros_args):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Joy
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        rclpy.init(args=ros_args)
        self.rclpy = rclpy
        self.set_bool, self.trigger = SetBool, Trigger
        self.node = rclpy.create_node("kart_joy_config", namespace=namespace)
        self.lock = threading.Lock()
        self.data, self.raw = {}, None
        self.status_time = self.raw_time = 0.0
        self.clients = {}
        self.node.create_subscription(Joy, "joy/raw", self.on_raw, qos_profile_sensor_data)
        self.node.create_subscription(String, "joy/status", self.on_status, 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_raw(self, msg):
        try:
            generation = int(msg.header.frame_id.removeprefix("kart_joy_raw_"))
        except ValueError:
            return
        with self.lock:
            self.raw = dict(raw_axes=list(msg.axes), raw_buttons=list(msg.buttons), generation=generation)
            self.raw_time = time.monotonic()

    def on_status(self, msg):
        try:
            data = yaml.safe_load(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get("profile_path"), str):
                return
        except yaml.YAMLError:
            return
        with self.lock:
            self.data = data
            self.status_time = time.monotonic()

    def state(self):
        with self.lock:
            data = copy.deepcopy(self.data)
            fresh = time.monotonic() - self.status_time < 3.0
            valid_raw = bool(self.raw and time.monotonic() - self.raw_time < 0.5 and
                             self.raw["generation"] == data.get("generation"))
            data["online"] = fresh
            data["connected"] = bool(data.get("connected") and fresh and valid_raw)
            data.update(raw_axes=self.raw["raw_axes"][:] if valid_raw else [],
                        raw_buttons=self.raw["raw_buttons"][:] if valid_raw else [])
            if not fresh:
                data["error"] = "ROS node status is stale / ノードを確認してください"
            return data

    def wait(self):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.state().get("online"):
                return
            time.sleep(0.1)
        raise RuntimeError("No joy/status; start kart_joy_node and check namespace/DDS settings")

    def call(self, operation, enabled=None):
        state = self.state()
        if not state.get("online"):
            raise RuntimeError("ROS node is offline")
        node_name = state["node"].rstrip("/")
        service_name = f"{node_name}/{operation}"
        service = self.trigger if operation == "reload_profile" else self.set_bool
        client = self.clients.get(service_name)
        if client is None:
            client = self.node.create_client(service, service_name)
            self.clients[service_name] = client
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"Service unavailable: {service_name}")
        request = service.Request()
        if enabled is not None:
            request.data = enabled
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(4.0):
            raise RuntimeError("Service response timed out; inspect node status before retrying")
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response.message

    def get_profile(self):
        path = self.state().get("profile_path", "")
        if not path:
            raise ValueError("Node needs an absolute profile_path")
        return profile.load(path) if Path(path).exists() else profile.default_profile()

    def save(self, value):
        profile.validate(value)
        # Configuration mode must be acknowledged, not inferred from stale UI.
        self.call("configure", True)
        path = self.state()["profile_path"]
        if not Path(path).is_absolute():
            raise ValueError("Node profile_path must be absolute")
        profile.save(path, value)
        try:
            return self.call("reload_profile")
        except Exception as exc:
            raise RuntimeError(f"Saved {path}, but reload did not succeed: {exc}. Output remains in configuration mode.") from exc

    def close(self):
        # Never auto-resume output when the editor closes or crashes.
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def serve(bridge, port):
    from ament_index_python.packages import get_package_share_directory
    page = (Path(get_package_share_directory("kart_joy")) / "web/index.html").read_text()
    token = secrets.token_urlsafe(32)
    page = page.replace("__TOKEN__", token).encode()
    mutation_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, content, kind="application/json"):
            body = content if isinstance(content, bytes) else json.dumps(content, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            # Defend localhost endpoints against DNS rebinding.
            if self.headers.get("Host") not in (f"127.0.0.1:{port}", f"localhost:{port}"):
                self.reply(403, {"error": "Invalid Host"})
                return
            try:
                if self.path == "/":
                    self.reply(200, page, "text/html; charset=utf-8")
                elif self.path == "/api/state":
                    self.reply(200, bridge.state())
                elif self.path == "/api/profile":
                    self.reply(200, bridge.get_profile())
                else:
                    self.reply(404, {"error": "Not found"})
            except (ValueError, OSError, RuntimeError, yaml.YAMLError) as exc:
                self.reply(400, {"error": str(exc)})

        def do_POST(self):
            if not secrets.compare_digest(self.headers.get("X-Kart-Token", ""), token):
                self.reply(403, {"error": "Invalid editor token"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 65536:
                    raise ValueError("Request must be 1..65536 bytes")
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError("Request must be a JSON object")
                with mutation_lock:
                    if self.path == "/api/save":
                        message = bridge.save(data)
                    elif self.path == "/api/configure" and type(data.get("enabled")) is bool:
                        message = bridge.call("configure", data["enabled"])
                    else:
                        raise ValueError("Invalid request")
                self.reply(200, {"message": message})
            except (ValueError, OSError, RuntimeError, yaml.YAMLError) as exc:
                self.reply(400, {"error": str(exc)})

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"Joy editor: http://127.0.0.1:{port}/", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


def sample(bridge):
    state = bridge.state()
    if not state.get("connected"):
        raise ValueError("Controller not connected / 入力を取得できません")
    return state


def capture(bridge, kind, timeout=10):
    baseline = sample(bridge)
    print("対象の入力を操作してください（10秒）。", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = sample(bridge)
        binding = profile.detect(baseline, state, kind)
        if binding:
            return binding, baseline, state
        time.sleep(0.02)
    raise ValueError("Capture timed out")


def cui(bridge):
    value = bridge.get_profile()
    saved_value = copy.deepcopy(value)
    bridge.call("configure", True)
    print("設定中: /joyは中立。終了しても自動復帰しません。")
    while True:
        print("\naxis / button / device / show / save / resume / quit")
        try:
            command = input("> ").strip()
            if command == "quit":
                return
            if command == "show":
                print(yaml.safe_dump(value, sort_keys=False))
                state = bridge.state()
                print(state.get("device"), state.get("error"))
                if state.get("connected"):
                    print("Axes:", {a["name"]: round(state["raw_axes"][a["code"]], 3)
                                    for a in state["axes"]})
                    print("Pressed:", [b["name"] for b in state["buttons"]
                                       if state["raw_buttons"][b["code"]]])
            elif command == "device":
                value["device"] = {key: sample(bridge)["device"][key]
                                   for key in ("vendor", "product", "name", "unique")}
                print("接続中の機体識別情報を取り込みました。")
            elif command in ("axis", "button"):
                kind = "axes" if command == "axis" else "buttons"
                print(" / ".join(value[kind]))
                name = input("割り当て先: ").strip()
                if name not in value[kind]:
                    raise ValueError("Unknown output name")
                if input("無効化する場合は disable、設定する場合はEnter: ").strip() == "disable":
                    value[kind][name]["code"] = -1
                    continue
                input("全て離してEnter → 対象の入力だけを操作: ")
                binding, baseline, detected = capture(bridge, kind)
                if kind == "buttons":
                    value[kind][name] = binding
                else:
                    item = copy.deepcopy(value[kind][name])
                    item.update(binding)
                    code = item["code"]
                    item["center"] = baseline["raw_axes"][code]
                    mode = input("bipolar（スティック）/ unipolar（トリガー） [bipolar]: ").strip() or "bipolar"
                    item["mode"] = mode
                    input("出力+1にしたい端まで倒したままEnter: ")
                    current = sample(bridge)
                    if current["generation"] != detected["generation"]:
                        raise ValueError("Controller changed during calibration")
                    item["positive"] = current["raw_axes"][code]
                    if mode == "bipolar":
                        input("出力-1にしたい端まで倒したままEnter: ")
                        current = sample(bridge)
                        if current["generation"] != detected["generation"]:
                            raise ValueError("Controller changed during calibration")
                        item["negative"] = current["raw_axes"][code]
                    item["deadzone"] = float(input("deadzone [0.05]: ").strip() or "0.05")
                    candidate = copy.deepcopy(value)
                    candidate[kind][name] = item
                    profile.validate(candidate)
                    value = candidate
                print("割り当て済み。saveで保存・反映。")
            elif command == "save":
                print(bridge.save(value))
                saved_value = copy.deepcopy(value)
            elif command == "resume":
                if value != saved_value:
                    raise ValueError("未保存の編集があります。saveしてから再開してください。")
                print(bridge.call("configure", False))
                return
        except (ValueError, RuntimeError, OSError) as exc:
            print(f"ERROR: {exc}")


def main():
    parser = argparse.ArgumentParser(description="kart joy GUI/CUI configuration")
    parser.add_argument("mode", choices=("gui", "cui", "init", "check"))
    parser.add_argument("--profile", help="YAML path for init/check")
    parser.add_argument("--namespace", default="/")
    parser.add_argument("--port", type=int, default=8766)
    args, ros_args = parser.parse_known_args()
    if args.mode in ("init", "check"):
        if not args.profile:
            parser.error("--profile is required")
        if args.mode == "init":
            if Path(args.profile).exists():
                parser.error("Profile already exists; refusing to overwrite")
            profile.save(args.profile, profile.default_profile())
        else:
            profile.load(args.profile)
        print(args.profile)
        return
    bridge = Bridge(args.namespace, ros_args)
    try:
        bridge.wait()
        serve(bridge, args.port) if args.mode == "gui" else cui(bridge)
    except (KeyboardInterrupt, EOFError):
        print("\n終了。設定モードからの自動復帰は行いません。")
    finally:
        bridge.close()
