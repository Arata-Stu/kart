"""Portable ownership/routing tests."""

import unittest
from pathlib import Path

from kart_bringup.container_plan import plan

CONFIG = Path(__file__).resolve().parents[1] / "config/localization/containers.json"


class ContainerPlanTest(unittest.TestCase):
    def test_default_two_owned_containers(self):
        targets, created = plan(CONFIG, {})
        self.assertEqual(len(created), 2)
        self.assertEqual(targets["vslam"], "/kart_vslam_container")

    def test_external_namespaced_shared_container(self):
        targets, created = plan(
            CONFIG,
            {
                "vslam_container": "/robot/perception",
                "vgl_container": "/robot/perception",
                "create_vslam_container": False,
                "create_vgl_container": "false",
            },
        )
        self.assertEqual(created, [])
        self.assertEqual(targets["vslam"], targets["vgl"])

    def test_shared_container_created_once(self):
        _, created = plan(
            CONFIG, {"vslam_container": "shared", "vgl_container": "/shared"}
        )
        self.assertEqual(len(created), 1)

    def test_mixed_ownership_rejected(self):
        with self.assertRaisesRegex(ValueError, "ownership"):
            plan(
                CONFIG,
                {
                    "vslam_container": "shared",
                    "vgl_container": "shared",
                    "create_vgl_container": False,
                },
            )

    def test_one_owned_one_external(self):
        targets, created = plan(
            CONFIG,
            {"vgl_container": "/robot/external", "create_vgl_container": "false"},
        )
        self.assertEqual(len(created), 1)
        self.assertEqual(targets["vgl"], "/robot/external")

    def test_invalid_flag_rejected(self):
        with self.assertRaisesRegex(ValueError, "true or false"):
            plan(CONFIG, {"create_vslam_container": "maybe"})
