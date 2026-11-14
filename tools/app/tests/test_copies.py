import tempfile
import os
import unittest
from pathlib import Path

from kart_studio.copies import copy_map
from kart_studio.jobs import Cancelled, Job, Jobs
from kart_studio.maps import Maps
from kart_studio.server import Studio
from kart_studio.storage import atomic_json


class CopyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.maps = Maps(root / "map")
        source = self.maps.root / "course"
        source.mkdir()
        atomic_json(source / "snapshot.json", {"source": "immutable"})
        atomic_json(source / "cloud.json", {"points": [[1, 2, 3]]})
        atomic_json(
            source / "map.json",
            {
                "schema": "kart.hdmap.v2",
                "title": "course",
                "revision": 7,
                "updated": 0,
                "point_count": 1,
                "frame": "map",
                "lanes": [
                    {
                        "id": key,
                        "left": [[1, 2]],
                        "right": [],
                        "closed": False,
                        "custom": [],
                        "custom_speeds": [],
                        "lines": {"centerline": {"points": [[1, 2], [2, 2]]}},
                    }
                    for key in ("main", "shortcut")
                ],
            },
        )
        (source / "cuvslam_map").mkdir()
        (source / "cuvslam_map/data").write_bytes(b"vslam")
        (source / "exports").mkdir()
        (source / "exports/old").write_text("old revision")
        self.jobs = Jobs(root / "jobs")
        self.job = Job(self.jobs.root, "copy", [])

    def copy(self, revision=7):
        return copy_map(self.maps, self.job, "course", "course_v2", revision)

    def test_independent_complete_variant(self):
        original = self.maps.load("course")
        result = self.copy()
        self.assertEqual(result["map"], "course_v2")
        clone = self.maps.load("course_v2")
        self.assertEqual(clone["lanes"], original["lanes"])
        self.assertEqual(clone["revision"], 1)
        self.assertEqual(clone["copied_from"]["revision"], 7)
        for file in ("snapshot.json", "cloud.json", "cuvslam_map/data"):
            a, b = (
                self.maps.folder("course") / file,
                self.maps.folder("course_v2") / file,
            )
            self.assertEqual(a.read_bytes(), b.read_bytes())
            self.assertNotEqual(a.stat().st_ino, b.stat().st_ino)
        clone["lanes"][0]["left"] = []
        self.maps.save("course_v2", clone)
        self.assertEqual(self.maps.load("course"), original)
        self.assertFalse((self.maps.folder("course_v2") / "exports").exists())
        self.maps.trash("course", 7)
        self.assertEqual(
            (self.maps.folder("course_v2") / "cuvslam_map/data").read_bytes(), b"vslam"
        )

    def test_stale_and_collision(self):
        with self.assertRaises(ValueError):
            self.copy(6)
        self.copy()
        with self.assertRaises(ValueError):
            self.copy()

    def test_cancel_and_external_symlink_leave_no_partial_map(self):
        self.job.cancel.set()
        with self.assertRaises(Cancelled):
            self.copy()
        self.job.cancel.clear()
        (self.maps.folder("course") / "cuvslam_map/link").symlink_to(self.maps.root.parent)
        with self.assertRaisesRegex(ValueError, 'cuvslam_map/link'):
            self.copy()
        self.assertFalse((self.maps.root / "course_v2").exists())
        self.assertEqual(list(self.maps.root.glob(".copy-*")), [])

    def test_official_latest_and_absolute_internal_links_are_independent(self):
        source = self.maps.folder('course')
        run = source / 'official/2026-11-15-run'
        run.mkdir(parents=True)
        (run / 'run.log').write_text('official run')
        (run.parent / 'latest').symlink_to(run.name, target_is_directory=True)
        (run.parent / 'absolute_alias').symlink_to(run, target_is_directory=True)
        (source / 'cuvslam_map/data_alias').symlink_to('data')
        (source / 'map_alias.json').symlink_to(source / 'map.json')
        self.copy()
        clone = self.maps.folder('course_v2')
        for path in ('official/latest', 'official/absolute_alias', 'cuvslam_map/data_alias', 'map_alias.json'):
            link = clone / path
            self.assertTrue(link.is_symlink())
            self.assertFalse(Path(os.readlink(link)).is_absolute())
            self.assertTrue(link.resolve(strict=True).is_relative_to(clone))
        self.maps.trash('course', 7)
        self.assertEqual((clone / 'official/latest/run.log').read_text(), 'official run')
        self.assertEqual((clone / 'official/absolute_alias/run.log').read_text(), 'official run')
        self.assertEqual((clone / 'cuvslam_map/data_alias').read_bytes(), b'vslam')
        from kart_studio.storage import read_json
        self.assertEqual(read_json(clone / 'map_alias.json')['revision'], 1)

    def test_broken_cyclic_and_excluded_links_report_paths_and_clean_up(self):
        source = self.maps.folder('course')
        for target in ('missing', 'alias', '.', 'exports/old'):
            with self.subTest(target=target):
                link = source / 'alias'
                link.symlink_to(target)
                try:
                    with self.assertRaisesRegex(ValueError, 'alias'):
                        self.copy()
                    self.assertFalse((self.maps.root / 'course_v2').exists())
                    self.assertEqual(list(self.maps.root.glob('.copy-*')), [])
                finally:
                    link.unlink()

    @unittest.skipUnless(hasattr(os, 'mkfifo'), 'POSIX named pipe required')
    def test_special_file_reports_its_path_and_cleans_up(self):
        pipe = self.maps.folder('course') / 'official_pipe'
        os.mkfifo(pipe)
        with self.assertRaisesRegex(ValueError, 'official_pipe'):
            self.copy()
        self.assertFalse((self.maps.root / 'course_v2').exists())
        self.assertEqual(list(self.maps.root.glob('.copy-*')), [])

    def test_api_locks_both_source_and_destination(self):
        studio = Studio.__new__(Studio)
        studio.maps, studio.jobs = self.maps, self.jobs
        for resource in ("map:course", "map:course_v2"):
            with self.jobs.guard(resource), self.assertRaises(ValueError):
                studio.post(
                    "/api/copy-map",
                    {"id": "course", "name": "course_v2", "revision": 7},
                )
