"""fzf selection contract; no ROS, Docker or vehicle commands."""
import importlib.util
import os
import io
import sys
import json
import tempfile
from contextlib import redirect_stdout
import yaml
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "bringup_tui", Path(__file__).resolve().parents[1] / "lib/bringup.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def with_edits(choose, *actions):
    actions = iter(actions)
    def select(title, values, *args, **kwargs):
        if title.startswith('設定一覧'):
            action = next(actions)
            assert action in values
            return action
        return choose(title, values, *args, **kwargs)
    return select


class FzfTests(unittest.TestCase):
    def test_drive_vslam_then_recovery_renders_each_profiles_labels(self):
        from kart_bringup.multicam import LABELS as multicam_labels
        from kart_bringup.recovery import LABELS as recovery_labels
        for recovery in ('off', 'infra_imu', 'rgb_imu'):
            with self.subTest(recovery=recovery), tempfile.TemporaryDirectory() as folder:
                hdmap = Path(folder) / 'hd_map.yaml'
                hdmap.write_text('{}')
                prompts = []

                def choose(title, values, label=str, default=0):
                    prompts.append(title)
                    # fzf renders every candidate before selecting one.
                    labels = [label(value) for value in values]
                    if title == '用途':
                        return 'drive'
                    if title == 'VSLAM速度・精度':
                        self.assertEqual(labels, list(multicam_labels.values()))
                    if title == '壁接触・スタック自動復帰':
                        self.assertEqual(labels, list(recovery_labels.values()))
                        self.assertEqual(values[default], 'off')
                        return recovery
                    return values[default]

                output = io.StringIO()
                argv = ['bringup.py', '--dry-run', '--no-bridge', '--no-evs',
                        '--rgb-profile', '640x480x30', '--infra-profile', '640x480x90',
                        '--imu-profile', 'high', '--no-tuning', '--auto-fixed-throttle=-1',
                        '--run-name', 'recovery_check', '--map-file', str(hdmap),
                        '--map-dir', folder, '--model-dir', folder]
                with (patch.object(sys, 'argv', argv),
                      patch.object(sys.stdin, 'isatty', return_value=True),
                      patch.object(module, 'choose', side_effect=with_edits(choose, 'multicam', 'recovery', 'start')),
                      patch('kart_bringup.mission.hdmap_choices', return_value=[('lane_001', 'centerline')]),
                      patch('kart_bringup.localization.resolve', return_value=(
                          {'cuvslam': {'tracking_mode': 0}}, {}, {}, {'input_shape': [1, 3, 480, 640]})),
                      redirect_stdout(output)):
                    module.main()
                self.assertLess(prompts.index('VSLAM速度・精度'), prompts.index('壁接触・スタック自動復帰'))
                self.assertIn(f'recovery:={recovery}', output.getvalue())
                self.assertIn('mission.launch.py', output.getvalue())

    def test_infra_vga_90_selection_reaches_launch(self):
        for mode in ("collect", "environment"):
            def choose(title, values, *args, **kwargs):
                if title == "用途":
                    self.assertEqual(values, ["record", "drive", "e2e", "eval", "esc-trial"])
                    return "record"
                if title == "記録内容":
                    self.assertEqual(values, ["collect", "environment"])
                    return mode
                if "解像度 × FPS" in title:
                    if title.startswith("RGB"):
                        self.assertNotIn("640x480x90", values)
                        return "640x480x60"
                    self.assertIn("640x480x90", values)
                    return "640x480x90"
                return values[0]
            output = io.StringIO()
            with (patch.object(sys, "argv", ["bringup.py", "--dry-run", "--no-bridge", "--no-evs",
                                             "--run-name", "vga_check", "--auto-fixed-throttle=-1"]),
                  patch.object(sys.stdin, "isatty", return_value=True),
                  patch.object(module, "choose", side_effect=with_edits(choose, 'rgb', 'infra', 'start')), redirect_stdout(output)):
                module.main()
            self.assertIn(f"mode:={mode}", output.getvalue())
            self.assertNotIn("mode:=record", output.getvalue())
            self.assertIn("rgb_profile:=640x480x60", output.getvalue())
            self.assertIn("infra_profile:=640x480x90", output.getvalue())

    def test_additional_camera_profiles_reach_tui_and_launch_arguments(self):
        expected = {f"{size}x{fps}" for size in ("480x270", "640x360") for fps in (30, 60, 90)}
        for mode in ("collect", "environment"):
            for profile in sorted(expected):
                with self.subTest(mode=mode, profile=profile):
                    selected = []
                    def choose(title, values, *args, **kwargs):
                        if title == "用途":
                            self.assertEqual(values, ["record", "drive", "e2e", "eval", "esc-trial"])
                            return "record"
                        if title == "記録内容":
                            self.assertEqual(values, ["collect", "environment"])
                            return mode
                        if "解像度 × FPS" in title:
                            self.assertTrue(expected.issubset(set(values)))
                            if title.startswith("RGB"):
                                self.assertNotIn("848x480x90", values)
                            else:
                                self.assertIn("848x480x90", values)
                            selected.append(title)
                            return profile
                        return values[0]
                    output = io.StringIO()
                    with (patch.object(sys, "argv", ["bringup.py", "--dry-run", "--no-bridge", "--no-evs",
                                                     "--run-name", "profile_check", "--auto-fixed-throttle=-1"]),
                          patch.object(sys.stdin, "isatty", return_value=True),
                          patch.object(module, "choose", side_effect=with_edits(choose, 'rgb', 'infra', 'start')), redirect_stdout(output)):
                        module.main()
                    self.assertEqual(len(selected), 2)
                    for key in ("rgb_profile", "infra_profile"):
                        self.assertIn(f"{key}:={profile}", output.getvalue())

    def test_848_by_100_is_infra_only_in_tui(self):
        def choose(title, values, *args, **kwargs):
            if title == '用途': return 'record'
            if title == '記録内容': return 'collect'
            if '解像度 × FPS' in title:
                if title.startswith('RGB'):
                    self.assertNotIn('848x100x100', values)
                    return '0'
                self.assertIn('848x100x100', values)
                return '848x100x100'
            return values[0]
        output = io.StringIO()
        with (patch.object(sys, 'argv', ['bringup.py', '--dry-run', '--no-bridge', '--no-evs',
                                        '--run-name', 'narrow', '--auto-fixed-throttle=-1']),
              patch.object(sys.stdin, 'isatty', return_value=True),
              patch.object(module, 'choose', side_effect=with_edits(choose, 'rgb', 'infra', 'start')), redirect_stdout(output)):
            module.main()
        self.assertIn('infra_profile:=848x100x100', output.getvalue())

    def test_imu_selection_reaches_mission_command(self):
        for mode in ("collect", "environment"):
            for profile in ("low", "high", "off"):
                def choose(title, values, *args, **kwargs):
                    if title == "用途":
                        self.assertEqual(values, ["record", "drive", "e2e", "eval", "esc-trial"])
                        return "record"
                    if title == "記録内容":
                        self.assertEqual(values, ["collect", "environment"])
                        return mode
                    if title == "IMU 取得レート":
                        self.assertEqual(values[kwargs["default"]], "")
                        return profile
                    return values[0]
                output = io.StringIO()
                with (patch.object(sys, "argv", ["bringup.py", "--dry-run", "--no-bridge", "--no-evs",
                                                 "--run-name", "imu_check", "--auto-fixed-throttle=-1"]),
                      patch.object(sys.stdin, "isatty", return_value=True),
                      patch.object(module, "choose", side_effect=with_edits(choose, 'imu', 'start')), redirect_stdout(output)):
                    module.main()
                self.assertIn(f"imu_profile:={profile}", output.getvalue())

    def test_default_duplicate_labels_and_original_path(self):
        values = [Path("/maps/a b"), Path("/maps/a\nb")]
        def run(command, **kwargs):
            self.assertIn("--read0", command)
            self.assertIn("--with-nth=2..", command)
            self.assertNotIn("FZF_DEFAULT_OPTS", kwargs["env"])
            self.assertNotIn("FZF_DEFAULT_OPTS_FILE", kwargs["env"])
            rows = kwargs["input"].split("\0")
            self.assertTrue(rows[0].startswith("1\t"))
            return subprocess.CompletedProcess(command, 0, rows[0] + "\0")
        with patch.object(module.shutil, "which", return_value="/bin/fzf"),              patch.object(module.subprocess, "run", side_effect=run),              patch.dict(os.environ, {"FZF_DEFAULT_OPTS": "--filter=other", "FZF_DEFAULT_OPTS_FILE": "/tmp/options"}):
            self.assertIs(module.choose("Map", values, label=lambda _: "同名", default=1), values[1])

    def test_cancel_and_invalid_output_never_select(self):
        with patch.object(module.shutil, "which", return_value="/bin/fzf"):
            for code in (1, 130, -2):
                with patch.object(module.subprocess, "run", return_value=subprocess.CompletedProcess([], code, "")):
                    with self.assertRaises(KeyboardInterrupt):
                        module.choose("Mode", ["collect", "drive"])
            for code, output in ((2, ""), (0, "not a candidate\0")):
                with patch.object(module.subprocess, "run", return_value=subprocess.CompletedProcess([], code, output)):
                    with self.assertRaises(ValueError):
                        module.choose("Mode", ["collect", "drive"])

    def test_missing_dependency_or_candidates(self):
        with patch.object(module.shutil, "which", return_value=None):
            with self.assertRaisesRegex(ValueError, "apt-get install"):
                module.choose("Mode", ["collect"])
            with self.assertRaisesRegex(ValueError, "候補がありません"):
                module.choose("Map", [])


class FreshDiagnosticTests(unittest.TestCase):
    def test_interactive_fourth_mode_skips_maps_and_models(self):
        config = module.ROOT / "ros2_ws/src/kart_bringup/config"
        flow = json.loads((config / "localization/workflow.json").read_text())
        topics = [flow[k] for k in ("left_image", "right_image", "left_info", "right_info")] + ["/tf_static"]
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / "run"
            bag.mkdir()
            rows = [{"topic_metadata": {"name": t}, "message_count": 10} for t in topics]
            (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {"topics_with_message_count": rows}}))
            titles = []
            def choose(title, values, *args, **kwargs):
                titles.append(title)
                if title == "用途": return "eval"
                if title == "オフライン解析":
                    self.assertEqual(values, ["vslam", "vgl", "fresh", "e2e_latency"])
                    return "fresh"
                if title == "追跡方式": return "0"
                if title == "VSLAM速度・精度":
                    self.assertEqual(values,['0','1','2'])
                    self.assertEqual(kwargs['default'],1)
                    self.assertIn('精度重視',args[0]('2'))
                    return '2'
                if title == "再生rosbag": return values[0]
                self.fail(f"Unexpected selection: {title}")
            output = io.StringIO()
            with patch.object(sys, "argv", ["bringup.py", "--dry-run", "--record-dir", folder]), patch.object(sys.stdin, "isatty", return_value=True), patch.object(module, "choose", side_effect=with_edits(choose, 'multicam', 'start')), redirect_stdout(output):
                module.main()
            text = output.getvalue()
            self.assertIn("sim_vslam.launch.py", text)
            self.assertIn("visualize:=true", text)
            self.assertIn("rviz:=true", text)
            self.assertIn("tracking_mode:=0", text)
            self.assertNotIn("map_dir:=", text)
            self.assertEqual(titles, ["用途", "オフライン解析", "追跡方式", "再生rosbag", "VSLAM速度・精度"])
            self.assertIn('multicam_mode:=2',text)
            with patch.object(sys, "argv", ["bringup.py", "--mode", "eval", "--eval-localization", "fresh", "--bag", str(bag), "--tracking-mode", "1", "--dry-run"]):
                with self.assertRaisesRegex(ValueError, "/realsense/imu"):
                    module.main()

    def test_interactive_vio_uses_imu_without_vgl_or_map(self):
        topics=['/realsense/infra1/image_rect_raw','/realsense/infra2/image_rect_raw',
                '/realsense/infra1/camera_info','/realsense/infra2/camera_info','/realsense/imu','/tf_static']
        with tempfile.TemporaryDirectory() as folder:
            bag=Path(folder)/'real_bag';bag.mkdir()
            rows=[{'topic_metadata':{'name':t},'message_count':10} for t in topics]
            (bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'topics_with_message_count':rows}}))
            def choose(title,values,*args,**kwargs):
                if title=='用途':return 'eval'
                if title=='オフライン解析':
                    self.assertIn('fresh',values)
                    return 'fresh'
                if title=='追跡方式':return '1'
                if title=='VSLAM速度・精度':return '1'
                if title=='再生rosbag':return values[0]
                self.fail(f'Unexpected selection: {title}')
            output=io.StringIO()
            with patch.object(sys,'argv',['bringup.py','--dry-run','--record-dir',folder,'--base-frame','camera_link','--imu-frame','camera_imu_optical_frame']),patch.object(sys.stdin,'isatty',return_value=True),patch.object(module,'choose',side_effect=with_edits(choose, 'multicam', 'start')),redirect_stdout(output):
                module.main()
            text=output.getvalue()
            self.assertIn('sim_vslam.launch.py',text)
            self.assertIn('tracking_mode:=1',text)
            self.assertIn('base_frame:=camera_link',text)
            self.assertIn('imu_frame:=camera_imu_optical_frame',text)
            self.assertIn('multicam_mode:=1',text)
            self.assertIn('rviz:=true',text)
            self.assertNotIn('map_dir:=',text)
            self.assertNotIn('enable_vgl:=true',text)

    def test_saved_map_tracking_selection_and_explicit_override(self):
        from kart_bringup import mission, localization
        for localization_kind, mode, explicit, has_imu in (
            (kind, mode, explicit, has_imu)
            for kind in ('vslam', 'vgl')
            for mode, explicit, has_imu in [('0', False, False), ('1', False, True),
                                           ('0', True, False), ('1', True, True), ('1', False, False)]
        ):
            with self.subTest(localization=localization_kind, mode=mode, explicit=explicit, has_imu=has_imu), tempfile.TemporaryDirectory() as folder:
                bag = Path(folder) / 'bag'
                bag.mkdir()
                topics = ['/realsense/infra1/image_rect_raw', '/realsense/infra2/image_rect_raw',
                          '/realsense/infra1/camera_info', '/realsense/infra2/camera_info', '/tf_static']
                if has_imu:
                    topics.append('/realsense/imu')
                (bag / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
                    'topics_with_message_count': [{'topic_metadata': {'name': t}, 'message_count': 10} for t in topics]}}))
                titles = []
                def choose(title, values, *args, **kwargs):
                    titles.append(title)
                    if title == '用途': return 'eval'
                    if title == 'オフライン解析':
                        self.assertEqual(values, ['vslam', 'vgl', 'fresh', 'e2e_latency'])
                        return localization_kind
                    if title == '追跡方式': return mode
                    return values[0]
                def discover(root, kind):
                    return [bag] if kind == 'bag' else [Path('/map/hdmap.yaml')] if kind == 'hdmap' else [Path('/map/course')]
                argv = ['bringup.py', '--dry-run', '--record-dir', folder]
                if explicit:
                    argv += ['--tracking-mode', mode]
                output = io.StringIO()
                with patch.object(sys, 'argv', argv), patch.object(sys.stdin, 'isatty', return_value=True), patch.object(module, 'choose', side_effect=with_edits(choose, 'start', 'cancel')), patch.object(mission, 'discover', side_effect=discover), patch.object(mission, 'hdmap_choices', return_value=[('lane_001', 'centerline')]), patch.object(localization, 'resolve', return_value=({'cuvslam': {'tracking_mode': int(mode)}}, None, None, {'input_shape': [1, 3, 360, 640]})) as resolve, patch('kart_bringup.replay.validate_bag_shape', return_value={}) as shape_check, redirect_stdout(output):
                    if mode == '1' and not has_imu:
                        errors = io.StringIO()
                        from contextlib import redirect_stderr
                        with redirect_stderr(errors):
                            module.main()
                        self.assertIn('/realsense/imu', errors.getvalue())
                        continue
                    module.main()
                self.assertEqual('追跡方式' in titles, not explicit)
                self.assertEqual(resolve.call_args.kwargs['overrides']['tracking_mode'], mode)
                self.assertEqual(resolve.call_args.kwargs['enable_vgl'], localization_kind == 'vgl')
                if localization_kind == 'vgl':
                    shape_check.assert_called_once_with(module.ROOT / 'ros2_ws/src/kart_bringup/config', str(bag), [1, 3, 360, 640])
                else:
                    shape_check.assert_not_called()
                self.assertIn(f'enable_vgl:={str(localization_kind == "vgl").lower()}', output.getvalue())
                self.assertIn(f'tracking_mode:={mode}', output.getvalue())

    def test_saved_map_vio_selects_map_and_replays_imu(self):
        with tempfile.TemporaryDirectory() as folder:
            bag=Path(folder)/'bag';bag.mkdir()
            topics=['/realsense/infra1/image_rect_raw','/realsense/infra2/image_rect_raw','/realsense/infra1/camera_info','/realsense/infra2/camera_info','/realsense/imu','/tf_static']
            (bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'topics_with_message_count':[{'topic_metadata':{'name':t},'message_count':10} for t in topics]}}))
            def choose(title,values,*args,**kwargs):
                if title=='用途':return 'eval'
                if title=='オフライン解析':
                    self.assertIn('vslam',values);return 'vslam'
                if title=='追跡方式':return '1'
                return values[0]
            def discover(root,kind):
                return [bag] if kind=='bag' else [Path('/map/hdmap.yaml')] if kind=='hdmap' else [Path('/map/course')]
            from kart_bringup import mission,localization
            output=io.StringIO()
            with patch.object(sys,'argv',['bringup.py','--dry-run','--record-dir',folder,'--imu-frame','camera_imu_optical_frame']),patch.object(sys.stdin,'isatty',return_value=True),patch.object(module,'choose',side_effect=with_edits(choose, 'start')),patch.object(mission,'discover',side_effect=discover),patch.object(mission,'hdmap_choices',return_value=[('lane_001','centerline')]),patch.object(localization,'resolve',return_value=({'cuvslam':{'tracking_mode':1}},None,None,{'input_shape':None})) as resolve,redirect_stdout(output):
                module.main()
            text=output.getvalue()
            self.assertIn('evaluation.launch.py',text)
            self.assertIn('tracking_mode:=1',text)
            self.assertIn('map_dir:=/map/course',text)
            self.assertIn('enable_vgl:=false',text)
            self.assertIn('imu_frame:=camera_imu_optical_frame',text)
            self.assertEqual(resolve.call_args.kwargs['overrides']['tracking_mode'],'1')
            self.assertNotIn('multicam_mode', resolve.call_args.kwargs['overrides'])
            self.assertNotIn('multicam_mode:=', text)


if __name__ == "__main__":
    unittest.main()
