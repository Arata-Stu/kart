import json
import math
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_studio.geometry import Corridor, centerline
from kart_studio.jobs import Jobs
from kart_studio.lines import DEFAULTS, generate, profile
from kart_studio.maps import Maps
from kart_studio.remote_agent import execute as remote_execute
from kart_studio.server import Handler
from kart_studio.snapshots import normalize
from kart_studio.storage import atomic_json, within
from kart_studio.transfers import profile as connection
from kart_studio.transfers import request


def track():
    def circle(r):
        return [
            [r * math.cos(i * math.tau / 20), r * math.sin(i * math.tau / 20)]
            for i in range(20)
        ]

    return dict(left=circle(3), right=circle(5), closed=True, custom=circle(4))


class GeometryTests(unittest.TestCase):
    def test_closed_bounds_phase_and_direction(self):
        doc = track()
        doc["right"] = list(reversed(doc["right"][7:] + doc["right"][:7]))
        points, widths, corridor = centerline(doc["left"], doc["right"], True, 0.5)
        self.assertGreater(len(points), 30)
        self.assertTrue(all(3.8 < math.hypot(*p) < 4.1 for p in points))
        self.assertTrue(all(min(w) > 0.7 for w in widths))
        corridor.validate(points, 0.14)

    def test_open_reversed_right_and_speed_limits(self):
        doc = dict(
            left=[[0, 1], [5, 1], [10, 1]],
            right=[[10, -1], [5, -1], [0, -1]],
            closed=False,
            custom=[[0, 0], [10, 0]],
        )
        line = generate(doc, "raceline", {"spacing": 0.5})
        rows = line["profile"]
        self.assertEqual(rows[0][5], 0)
        self.assertEqual(rows[-1][5], 0)
        self.assertTrue(all(abs(p[1]) < 1e-6 for p in line["points"]))
        self.assertTrue(all(-2.5 - 1e-6 <= r[6] <= 1.5 + 1e-6 for r in rows))
        self.assertTrue(all(r[5] <= 3 for r in rows))

    def test_raceline_clearance_and_curvature_improvement(self):
        doc = track()
        initial = generate(doc, "centerline", {"spacing": 0.5})
        result = generate(doc, "raceline", {"spacing": 0.5})
        corridor = Corridor(doc["left"], doc["right"], True)
        corridor.validate(result["points"], 0.14)
        baseline = profile(initial["points"], True, DEFAULTS)
        self.assertLess(
            sum(r[4] ** 2 for r in result["profile"]), sum(r[4] ** 2 for r in baseline)
        )
        self.assertTrue(
            all(r[5] ** 2 * abs(r[4]) <= 2.5 + 1e-6 for r in result["profile"])
        )

    def test_customline_rejects_inner_shortcut(self):
        doc = track()
        doc["custom"] = [[4, 0], [0, 4], [-4, 0], [0, -4]]
        with self.assertRaisesRegex(ValueError, "境界"):
            generate(doc, "customline", {})

    def test_crossing_boundaries_and_narrow_corridor(self):
        with self.assertRaises(ValueError):
            centerline([[0, 1], [2, -1]], [[0, -1], [2, 1]], False)
        with self.assertRaises(ValueError):
            generate(
                dict(
                    left=[[0, 0.1], [4, 0.1]],
                    right=[[0, -0.1], [4, -0.1]],
                    closed=False,
                ),
                "raceline",
                {},
            )

    def test_valid_customline_and_cancel(self):
        doc = track()
        result = generate(doc, "customline", {"spacing": 0.5})
        self.assertGreater(len(result["profile"]), 20)

        def cancel():
            raise InterruptedError("stop")

        with self.assertRaises(InterruptedError):
            generate(doc, "raceline", {"spacing": 0.5}, cancel)


class StorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.maps = Maps(self.root / "map")
        source = self.root / "snapshot.json"
        atomic_json(
            source,
            dict(
                schema="kart.snapshot.v1",
                frame="map",
                points=[[0, 0, 0], [1, 1, 1]],
                provenance={},
            ),
        )
        self.maps.import_snapshot("course", source)

    def tearDown(self):
        self.temp.cleanup()

    def test_revision_invalidation_and_export(self):
        d = self.maps.load("course")
        d["lanes"][0].update(track())
        d = self.maps.save("course", d)
        self.maps.generate(
            "course", "centerline", {"spacing": 0.5}, d["revision"], lambda: None
        )
        with self.assertRaisesRegex(ValueError, "更新"):
            self.maps.save("course", d)
        path = self.maps.export("course")
        self.assertTrue((path / "centerline.csv").is_file())
        yaml = json.loads((path / "hd_map.yaml").read_text())
        self.assertEqual(yaml["frame_id"], "map")
        d = self.maps.load("course")
        d["lanes"][0]["left"][0][0] += 0.1
        d = self.maps.save("course", d)
        self.assertEqual(d["lanes"][0]["lines"], {})
        self.assertTrue(path.is_dir())  # Previous export revision remains intact.

    def test_confined_paths_and_duplicate_import(self):
        with self.assertRaises(ValueError):
            within(self.maps.root, "../snapshot.json")
        (self.maps.root / "link").symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            within(self.maps.root, "link/snapshot.json")
        with self.assertRaises(ValueError):
            self.maps.import_snapshot("course", self.root / "snapshot.json")

    def test_snapshot_wrong_frame_and_missing_tf(self):
        with self.assertRaises(ValueError):
            normalize(dict(schema="kart.snapshot.v1", frame="odom", points=[[0, 0, 0]]))
        with self.assertRaises(ValueError):
            normalize(dict(landmarks=None))

    def test_stop_after_publication_keeps_success(self):
        jobs = Jobs(self.root / "jobs")

        def work(job):
            atomic_json(self.root / "published.json", {"complete": True})
            job.cancel.set()  # Stop arrived after the commit point.
            return {"published": True}

        value = jobs.start("commit", ["map:course"], work)
        deadline = time.monotonic() + 3
        while (
            jobs.jobs[value["id"]].status == "running" and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        self.assertEqual(jobs.jobs[value["id"]].status, "succeeded")

    def test_job_resource_lock_and_completion(self):
        jobs = Jobs(self.root / "jobs")
        gate = threading.Event()
        j = jobs.start("test", ["map:course"], lambda _: gate.wait(2))
        with self.assertRaises(ValueError):
            with jobs.guard("map:course"):
                pass
        gate.set()
        deadline = time.monotonic() + 3
        while jobs.jobs[j["id"]].status == "running" and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(jobs.jobs[j["id"]].status, "succeeded")
        with jobs.guard("map:course"):
            pass


class RemoteTests(unittest.TestCase):
    def test_profile_rejects_shell_syntax(self):
        base = dict(
            user="tamiya",
            host="jetson.local",
            port=22,
            record_root="/home/tamiya/record",
            map_root="/home/tamiya/map",
        )
        for key, value in [
            ("host", "-oProxyCommand=bad"),
            ("user", "x;echo"),
            ("record_root", "/tmp/../etc"),
            ("map_root", "/tmp/$(id)"),
        ]:
            with self.assertRaises(ValueError):
                connection(dict(base, **{key: value}))
        with patch("kart_studio.transfers.subprocess.run") as run:
            run.return_value.stdout = '{"entries":[]}'
            run.return_value.returncode = 0
            request(connection(base), "browse", base["record_root"])
            args = run.call_args.args[0]
            self.assertIn("StrictHostKeyChecking=yes", args)
            self.assertIn("BatchMode=yes", args)
            self.assertEqual(args[-2], "tamiya@jetson.local")

    def test_remote_nested_browse_and_symlink_rejection(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "session" / "bag").mkdir(parents=True)
            (root / "session" / "bag" / "metadata.yaml").write_text("metadata")
            (root / "session" / "bag" / "data.mcap").write_bytes(b"1234")
            result = remote_execute(
                dict(root=temp, action="browse", relative="session")
            )
            self.assertTrue(result["entries"][0]["bag"])
            stat = remote_execute(
                dict(root=temp, action="stat", relative="session/bag")
            )
            self.assertEqual(stat["files"], 2)
            (root / "session" / "bag" / "link").symlink_to(root)
            with self.assertRaises(ValueError):
                remote_execute(dict(root=temp, action="stat", relative="session/bag"))
            with self.assertRaises(ValueError):
                remote_execute(dict(root=temp, action="browse", relative="../"))


class HttpTests(unittest.TestCase):
    def test_origin_and_host(self):
        handler = Handler.__new__(Handler)
        handler.server = type("Server", (), {"server_port": 8766})()
        handler.headers = {"Host": "127.0.0.1:8766", "Origin": "http://127.0.0.1:8766"}
        handler.trusted()
        handler.headers["Origin"] = "https://untrusted.example"
        with self.assertRaises(ValueError):
            handler.trusted()
        handler.headers = {"Host": "example.com:8766"}
        with self.assertRaises(ValueError):
            handler.trusted()


if __name__ == "__main__":
    unittest.main()
