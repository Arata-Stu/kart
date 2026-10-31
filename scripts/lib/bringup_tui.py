"""Editable startup configuration. No ROS execution and no persistent defaults."""
from copy import deepcopy
import json
import math
from pathlib import Path

import yaml

from kart_bringup import mission
from kart_bringup.mission import CAMERA_PROFILES, camera_config, sensor_parameters
from bringup_assets import (matching_resolution, paired_maps, select_map_directory,
                            select_related, select_runtime_model)

EVALUATIONS = {
    'vslam': 'VSLAM（保存地図で自己位置推定）',
    'vgl': 'VSLAM＋VGL（準備済みbundleで自己位置推定）',
    'fresh': 'VSLAM（保存地図なし・新規SLAM）',
    'e2e_latency': 'E2E遅延（RGB bag → 推論 → 制御topic）',
}


def select_purpose(args, choose):
    labels = {'record': 'データ記録', 'drive': '地図走行（VSLAM＋VGL＋ライン追従）',
              'e2e': 'E2E（モデルによる走行）', 'eval': 'オフライン評価（記録bagを再生）',
              'esc-trial': 'ESC信号列試験（UI操作・地図／自己位置／Joy不要）'}
    args.mode = choose('用途', list(labels), labels.__getitem__)
    if args.mode == 'record':
        args.mode = choose('記録内容', ['collect', 'environment'],
                          lambda v: '走行データ' if v == 'collect' else '環境記録（Depth付き）')
    if args.mode == 'eval':
        args.eval_localization = args.eval_localization or choose(
            'オフライン解析', list(EVALUATIONS), EVALUATIONS.__getitem__)
        if args.eval_localization == 'e2e_latency':
            args.mode = 'e2e-latency'
        elif not args.tracking_mode:
            args.tracking_mode = choose('追跡方式', ['0', '1'],
                                       lambda v: 'stereo VO（IMUなし）' if v == '0' else 'VIO（IMUあり）')


class Editor:
    def __init__(self, args, config, root, choose):
        self.args = args
        self.config = Path(config)
        self.root = Path(root)
        self.choose = choose
        self.inferred = {}  # Only these values may be reset when changing assets.

    @property
    def live(self):
        return self.args.mode not in ('eval', 'e2e-latency')

    def params(self, relative, node):
        return yaml.safe_load((self.config / relative).read_text())[f'/**/{node}']['ros__parameters']

    def sensors(self):
        a = self.args
        return sensor_parameters(camera_config(self.config, a.mode), a.rgb_fps or '',
                                 a.infra_fps or '', a.rgb_profile, a.infra_profile, a.imu_profile)

    def pick(self, title, values, current=None, label=str):
        values = list(values)
        return self.choose(title, values, label, default=values.index(current) if current in values else 0)

    def infer(self, key, value):
        setattr(self.args, key, value)
        self.inferred[key] = value

    def reset_inferred(self):
        for key, value in self.inferred.items():
            if getattr(self.args, key) == value:
                setattr(self.args, key, False if key == 'evs' else '')
        self.inferred.clear()

    def prepare(self):
        try:
            self.assets()
        except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
            print(f'候補の準備: {error}\n設定一覧から選択し直してください。')

    def assets(self):
        a = self.args
        if a.mode in ('e2e', 'e2e-latency'):
            if not a.e2e_model_dir:
                a.e2e_model_dir = str(self.pick('E2E ONNXモデル（metadata付き）', sorted(set(
                    mission.discover(a.model_root, 'e2e') + mission.discover(self.root / 'e2e/models', 'e2e')))))
            if a.mode == 'e2e':
                from kart_e2e.contract import input_contract, routing_settings
                modality, _, _ = input_contract(a.e2e_model_dir)
                if modality == 'evs' and not a.no_evs and not a.evs:
                    self.infer('evs', True)
                if modality == 'evs' and not a.evs_tensor_config:
                    from kart_bringup.e2e import select_evs_tensor
                    self.infer('evs_tensor_config', select_evs_tensor(a.e2e_model_dir, ''))
                if routing_settings(a.e2e_model_dir):
                    self.maps(vslam_only=True, section=True)
        elif a.mode == 'drive' or (a.mode == 'eval' and a.eval_localization != 'fresh'):
            self.maps(vslam_only=a.mode == 'eval' and a.eval_localization == 'vslam')
        if not self.live and not a.bag:
            a.bag = str(self.pick('再生rosbag', mission.discover(a.record_dir, 'bag')))

    def maps(self, vslam_only=False, section=False):
        a = self.args
        if not a.map_file:
            a.map_file = str(select_map_directory(mission.discover(a.map_root, 'hdmap'), a.map_root, self.choose))
        if not section:
            options = [v for v in mission.hdmap_choices(a.map_file)
                       if (not a.lane_id or v[0] == a.lane_id) and (not a.line_type or v[1] == a.line_type)]
            if len(options) == 1:
                a.lane_id, a.line_type = options[0]
                print('lane / 追従ライン:', ' / '.join(options[0]), '（自動選択）')
            else:
                a.lane_id, a.line_type = self.pick('lane / 追従ライン', options, label=lambda v: ' / '.join(v))
        if not a.map_dir:
            candidates = mission.discover(a.map_root, 'vslam' if vslam_only else 'bundle')
            matched = paired_maps(a.map_file, candidates, a.map_root)
            if a.mode == 'drive' and matched:
                # Existing or explicit camera profile is a preference among paired maps.
                compatible = matching_resolution(matched, self.sensors()['depth_module.infra_profile'])
                matched = compatible or matched
            a.map_dir = str(select_related('対応するVSLAM地図' if vslam_only else '対応するVGL bundle',
                                          candidates, matched, self.choose))
        if not vslam_only and not a.model_dir:
            models = sorted(set(mission.discover(a.model_root, 'models') + mission.discover(a.map_root, 'models')))
            a.model_dir = str(select_runtime_model(a.map_dir, models, self.choose))
        if a.mode == 'drive' and not a.infra_profile and a.infra_fps is None:
            profile = json.loads((Path(a.map_dir) / 'vgl_profile.json').read_text())
            shape = profile['input_shape']
            if len(shape) != 4 or shape[:2] != [1, 3] or any(type(v) is not int or v <= 0 for v in shape):
                raise ValueError('VGL bundleのinput_shapeが不正です')
            width, height = shape[3], shape[2]
            fps = self.sensors()['depth_module.infra_profile'].split('x')[2]
            preferred = f'{width}x{height}x{fps}'
            choices = [p for p in CAMERA_PROFILES['Infra'] if p.startswith(f'{width}x{height}x')]
            if not choices:
                raise ValueError('VGL入力サイズに対応するInfra profileがありません')
            # Do not invent an FPS when the configured one is unsupported.
            selected = preferred if preferred in choices else self.pick('Infra 解像度 × FPS', choices)
            self.infer('infra_profile', selected)
            print('Infra:', selected, '（VGL bundleから推定）')

    def rows(self):
        a = self.args
        rows = {'start': '設定を検証（起動なし）' if a.dry_run else '設定を検証して起動'}
        if a.mode in ('drive', 'e2e', 'e2e-latency') or (a.mode == 'eval' and a.eval_localization != 'fresh'):
            rows['assets'] = f'地図・モデル: {a.map_file or "—"} / {a.e2e_model_dir or a.model_dir or "—"}'
            print('採用地図:', a.map_dir or 'なし', '/ HDMap:', a.map_file or 'なし')
            print('採用モデル:', a.e2e_model_dir or a.model_dir or 'なし')
        if a.mode in ('eval', 'e2e-latency'):
            rows['evaluation'] = '評価方式: ' + EVALUATIONS[a.eval_localization or 'vslam']
        if a.mode == 'drive' or (a.mode == 'eval' and a.eval_localization == 'vgl'):
            rows['bundle'] = 'VGL bundle: ' + (a.map_dir or '未選択')
            rows['model'] = 'VGLモデル: ' + (a.model_dir or '未選択')
        if a.mode in ('drive', 'eval') and a.map_file:
            rows['line'] = f'追従ライン: {a.lane_id} / {a.line_type}'
        if self.live:
            try:
                sensors = self.sensors()
                rgb = sensors['rgb_camera.color_profile'] if sensors['enable_color'] else 'OFF'
                infra = sensors['depth_module.infra_profile'] if sensors['enable_infra1'] else 'OFF'
                imu = (f"加速度{sensors['accel_fps']} / ジャイロ{sensors['gyro_fps']}Hz"
                       if sensors['enable_accel'] and sensors['enable_gyro'] else 'OFF')
                if a.mode == 'environment':
                    print('Depth:', sensors['depth_module.depth_profile'], '/ 記録内容: 環境記録')
            except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
                rgb, infra, imu = a.rgb_profile or 'YAML', a.infra_profile or 'YAML', a.imu_profile or 'YAML'
                print('センサ設定エラー:', error)
            rows.update(rgb=f'センサ / RGB: {rgb}', infra=f'センサ / Infra: {infra}', imu=f'センサ / IMU: {imu}',
                        evs=f'センサ / EVS: {"有効" if a.evs else "無効"}（詳細設定）')
            if a.evs:
                print('EVS詳細:', self.evs_summary())
            if a.mode in ('collect', 'environment'):
                limit = self.params('input/joy_manager.yaml', 'kart_joy_manager')['speed_scale'] if a.joy_max_throttle is None else a.joy_max_throttle
                rows['joy_limit'] = f'制御 / Joy最大スロットル: {limit}'
            rows['control'] = '制御 / AUTO: ' + self.control_summary()
            if a.mode in ('drive', 'e2e'):
                from kart_bringup.recovery import configuration, LABELS
                enabled, settings = configuration(self.config / 'recovery', a.recovery)
                rows['recovery'] = '補助 / 自動復帰: ' + LABELS[settings['sensor_profile'] if enabled else 'off']
            rows['foxglove'] = f'補助 / Foxglove: {"有効" if a.foxglove else "無効"}'
            if a.mode == 'drive':
                enabled = self.params('hdmap/hdmap.yaml', 'hdmap_server')['tuning_enabled'] if a.tuning is None else a.tuning
                rows['tuning'] = f'補助 / Studio調整: {"有効" if enabled else "無効"}'
            rows['record'] = f'記録: {a.record_dir} / {a.run_name or "run"}'
            rows['bridge'] = f'車両基板: {"無効" if a.no_bridge else a.device or "設定値（YAML）"}'
        else:
            rows['bag'] = f'再生bag: {a.bag or "未選択"}'
            rows['rate'] = f'再生倍率: {a.rate or "設定値（YAML）"}'
        if a.mode == 'eval':
            rows['tracking'] = f'追跡方式: {"VIO" if a.tracking_mode == "1" else "VO"}'
        if a.mode in ('drive', 'eval'):
            from kart_bringup.multicam import LABELS
            preset = 'sim/vslam.yaml' if a.mode == 'eval' and a.eval_localization == 'fresh' else 'localization/cuvslam.yaml'
            value = a.multicam_mode or str(self.params(preset, 'visual_slam')['multicam_mode'])
            rows['multicam'] = 'VSLAM速度・精度: ' + LABELS[value]
        rows['cancel'] = '終了（起動しない）'
        return rows

    def control_summary(self):
        a = self.args
        if a.mode == 'e2e':
            if not a.e2e_model_dir:
                return 'モデル未選択'
            from kart_e2e.contract import runtime_settings
            settings = runtime_settings(a.e2e_model_dir)
            value = settings['fixed_throttle'] if a.e2e_fixed_throttle is None else a.e2e_fixed_throttle
            return 'モデル出力' if a.e2e_throttle_mode == 'model' else f'モデル操舵＋固定スロットル {value}'
        value = self.params('input/joy_manager.yaml', 'kart_joy_manager')['auto_fixed_throttle'] if a.auto_fixed_throttle is None else a.auto_fixed_throttle
        return '通常AUTO' if value < 0 else f'固定スロットル {value}＋手動操舵'

    def evs_values(self):
        a = self.args
        backend = a.evs_backend or ('cuda_async' if a.evs_aggregation else
                  self.params('sensors/openeb_tensor_pipeline.yaml', 'tensor_pipeline')['tensor_backend'])
        path = Path(a.evs_tensor_config) if a.evs_tensor_config else self.config / f'sensors/openeb_tensor_{backend}.yaml'
        tensor = yaml.safe_load(path.expanduser().read_text())['/**/event_tensor']['ros__parameters']
        image = self.params('sensors/openeb_visualization.yaml', 'event_preprocessor')['event_image_enabled'] if a.evs_image is None else a.evs_image
        bias = a.evs_bias_file or self.params('sensors/openeb.yaml', 'event_camera_driver').get('bias_file', '') or '@default'
        return backend, a.evs_aggregation or tensor.get('event_aggregation', 'events'), image, bias

    def evs_summary(self):
        try:
            backend, aggregation, image, bias = self.evs_values()
            return f'{backend} / {aggregation} / 画像={image} / bias={bias} / Tensor={self.args.evs_tensor_config or "YAML"}'
        except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
            return f'設定を確認: {error}'

    def review(self):
        while True:
            try:
                rows = self.rows()
            except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
                print('設定読込みエラー:', error)
                rows = {'assets': '地図・モデルを選び直す', 'cancel': '終了'}
            action = self.pick('設定一覧（変更する項目を選択）', rows, label=rows.__getitem__)
            if action in ('start', 'cancel'):
                return action == 'start'
            before, inferred = deepcopy(self.args), dict(self.inferred)
            try:
                self.edit(action)
            except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
                self.args, self.inferred = before, inferred
                print(f'変更できません: {error}（変更前の設定を保持）')

    def edit(self, action):
        a = self.args
        if action == 'assets':
            self.reset_inferred()
            for key in ('map_file', 'map_dir', 'model_dir', 'lane_id', 'line_type', 'e2e_model_dir'):
                setattr(a, key, '')
            self.assets()
        elif action == 'evaluation':
            choice = self.pick('オフライン解析', EVALUATIONS, a.eval_localization, EVALUATIONS.__getitem__)
            if choice != a.eval_localization:
                for key in ('map_file', 'map_dir', 'model_dir', 'lane_id', 'line_type', 'e2e_model_dir',
                            'tracking_mode', 'multicam_mode', 'base_frame', 'imu_frame', 'latency_output'):
                    setattr(a, key, '')
                a.eval_localization = choice
                a.mode = 'e2e-latency' if choice == 'e2e_latency' else 'eval'
                if a.mode == 'eval':
                    self.edit('tracking')
                self.assets()
        elif action == 'bundle':
            candidates = mission.discover(a.map_root, 'bundle')
            matched = paired_maps(a.map_file, candidates, a.map_root)
            a.map_dir = str(self.pick('対応するVGL bundle', matched or candidates, Path(a.map_dir)))
            a.model_dir = ''
            self.reset_inferred()
            self.maps()
        elif action == 'model':
            from bringup_assets import paired_models
            candidates = sorted(set(mission.discover(a.model_root, 'models') + mission.discover(a.map_root, 'models')))
            candidates = sorted(paired_models(a.map_dir, candidates) or candidates,
                                key=lambda p: (Path(p).name != 'runtime_models', str(p)))
            a.model_dir = str(self.pick('VGLモデル（実行GPU用engine）', candidates, Path(a.model_dir)))
        elif action == 'line':
            a.lane_id, a.line_type = self.pick('lane / 追従ライン', mission.hdmap_choices(a.map_file),
                                              (a.lane_id, a.line_type), lambda v: ' / '.join(v))
        elif action in ('rgb', 'infra'):
            try:
                sensors = self.sensors()
            except ValueError:
                sensors = sensor_parameters(camera_config(self.config, a.mode))
            key, enabled = ('rgb_camera.color_profile', 'enable_color') if action == 'rgb' else ('depth_module.infra_profile', 'enable_infra1')
            current = sensors[key] if sensors[enabled] else '0'
            profiles = list(dict.fromkeys([current, *CAMERA_PROFILES['RGB' if action == 'rgb' else 'Infra'], '0']))
            value = self.pick(action.upper() + ' 解像度 × FPS（USB3 D455候補）', profiles, current,
                              lambda v: 'なし' if v == '0' else v.replace('x', ' × '))
            setattr(a, action + '_profile', value)
            setattr(a, action + '_fps', None)
            self.inferred.pop(action + '_profile', None)
        elif action == 'imu':
            a.imu_profile = self.pick('IMU 取得レート', ['', 'low', 'high', 'off'], a.imu_profile,
                lambda v: {'': '設定を保持（YAML）', 'low': '低Hz（加速度100 / ジャイロ200Hz）',
                           'high': '高Hz（加速度200 / ジャイロ400Hz）', 'off': 'なし'}[v])
        elif action == 'evs':
            self.edit_evs()
        elif action == 'joy_limit':
            current = self.params('input/joy_manager.yaml', 'kart_joy_manager')['speed_scale'] if a.joy_max_throttle is None else a.joy_max_throttle
            raw = input(f'Joy最大スロットル（前進・後退）0.01..1 [{current}] > ').strip()
            if raw:
                value = float(raw)
                if not math.isfinite(value) or not 0.01 <= value <= 1:
                    raise ValueError('Joy最大スロットルは0.01..1の有限値を指定してください')
                a.joy_max_throttle = value
        elif action == 'control':
            self.edit_control()
        elif action == 'recovery':
            from kart_bringup.recovery import CHOICES, LABELS, configuration
            enabled, settings = configuration(self.config / 'recovery', a.recovery)
            a.recovery = self.pick('壁接触・スタック自動復帰', CHOICES,
                                   settings['sensor_profile'] if enabled else 'off', LABELS.__getitem__)
        elif action in ('foxglove', 'tuning', 'bridge'):
            current = (not a.no_bridge if action == 'bridge' else a.foxglove if action == 'foxglove' else
                       self.params('hdmap/hdmap.yaml', 'hdmap_server')['tuning_enabled'] if a.tuning is None else a.tuning)
            value = self.pick({'foxglove': 'Foxglove Bridge', 'tuning': 'Map Studio 動的調整', 'bridge': '車両基板'}[action],
                              [False, True], current, lambda v: '有効' if v else '無効')
            setattr(a, 'no_bridge' if action == 'bridge' else action, not value if action == 'bridge' else value)
        elif action == 'record':
            while True:
                rows = {'back': '設定一覧へ戻る', 'name': f'記録名: {a.run_name or "run"}',
                        'directory': f'保存先: {a.record_dir}'}
                field = self.pick('記録設定', rows, label=rows.__getitem__)
                if field == 'back':
                    break
                if field == 'name':
                    a.run_name = input(f'run_name [{a.run_name or "run"}] > ').strip() or a.run_name or 'run'
                else:
                    value = input(f'記録保存先 [{a.record_dir}] > ').strip()
                    if value:
                        a.record_dir = str(Path(value).expanduser())
        elif action == 'bag':
            a.bag = str(self.pick('再生rosbag', mission.discover(a.record_dir, 'bag'), Path(a.bag)))
        elif action == 'rate':
            value = input(f'再生倍率 [{a.rate or "YAML"}] > ').strip()
            if value:
                if not math.isfinite(float(value)) or float(value) <= 0:
                    raise ValueError('再生倍率は正の有限値を指定してください')
                a.rate = value
        elif action == 'tracking':
            a.tracking_mode = self.pick('追跡方式', ['0', '1'], a.tracking_mode,
                                       lambda v: 'stereo VO（IMUなし）' if v == '0' else 'VIO（IMUあり）')
        elif action == 'multicam':
            from kart_bringup.multicam import LABELS
            preset = 'sim/vslam.yaml' if a.mode == 'eval' and a.eval_localization == 'fresh' else 'localization/cuvslam.yaml'
            current = a.multicam_mode or str(self.params(preset, 'visual_slam')['multicam_mode'])
            a.multicam_mode = self.pick('VSLAM速度・精度', LABELS, current, LABELS.__getitem__)

    def edit_control(self):
        a = self.args
        if a.mode == 'e2e':
            from kart_e2e.contract import model_contract, runtime_settings
            _, output = model_contract(a.e2e_model_dir)
            settings = runtime_settings(a.e2e_model_dir)
            a.e2e_throttle_mode = self.pick('E2Eスロットル', ['fixed', 'model'] if output == 'steer_throttle' else ['fixed'],
                                           a.e2e_throttle_mode or 'fixed',
                                           lambda v: '固定スロットル' if v == 'fixed' else 'モデル出力（可変スロットル）')
            if a.e2e_throttle_mode == 'model':
                a.e2e_fixed_throttle = None
            else:
                current = settings['fixed_throttle'] if a.e2e_fixed_throttle is None else a.e2e_fixed_throttle
                a.e2e_fixed_throttle = float(input(f'E2E固定スロットル 0..{settings["max_throttle"]} [{current}] > ').strip() or current)
        else:
            settings = self.params('input/joy_manager.yaml', 'kart_joy_manager')
            settings = dict(settings)
            if a.joy_max_throttle is not None:
                settings['speed_scale'] = a.joy_max_throttle
            value = settings['auto_fixed_throttle'] if a.auto_fixed_throttle is None else a.auto_fixed_throttle
            mode = self.pick('AUTOスロットル', ['normal', 'fixed'], 'fixed' if value >= 0 else 'normal',
                             lambda v: '通常AUTO' if v == 'normal' else '固定値で収集（ステア手動）')
            if mode == 'normal':
                a.auto_fixed_throttle = -1.0
            else:
                current = value if value >= 0 else ''
                a.auto_fixed_throttle = float(input(f'固定スロットル 0..{settings["speed_scale"]} [{current}] > ').strip() or current)

    def edit_evs(self):
        from kart_bringup.evs import BACKENDS, BACKEND_LABELS, AGGREGATIONS, AGGREGATION_LABELS, DEFAULT_BIAS, bias_files
        a = self.args
        while True:
            rows = {'back': '設定一覧へ戻る', 'enabled': f'EVS: {"有効" if a.evs else "無効"}'}
            if a.evs:
                print('EVS詳細:', self.evs_summary())
                rows.update(backend='Tensor処理方式', aggregation='集計方式（cuda_async）', image='可視化画像', bias='bias file')
            action = self.pick('EVS 詳細設定', rows, label=rows.__getitem__)
            if action == 'back':
                return
            if action == 'enabled':
                a.evs = self.pick('EVS', [False, True], a.evs, lambda v: '有効' if v else '無効')
                a.no_evs = not a.evs
                self.inferred.pop('evs', None)
                if not a.evs:
                    # Disabled EVS must not leak inactive overrides into launch.
                    for key in ('evs_backend', 'evs_aggregation', 'evs_serial', 'evs_tensor_config', 'evs_bias_file'):
                        setattr(a, key, '')
                        self.inferred.pop(key, None)
                    a.evs_image = None
            elif action == 'backend':
                a.evs_backend = self.pick('EVS Tensor処理方式', BACKENDS, self.evs_values()[0], BACKEND_LABELS.__getitem__)
                if a.evs_backend != 'cuda_async':
                    a.evs_aggregation = ''
            elif action == 'aggregation':
                a.evs_aggregation = self.pick('EVS 集計方式', AGGREGATIONS, self.evs_values()[1], AGGREGATION_LABELS.__getitem__)
                a.evs_backend = 'cuda_async'
            elif action == 'image':
                a.evs_image = self.pick('EVS event image可視化', [False, True], self.evs_values()[2], lambda v: 'ON' if v else 'OFF')
            elif action == 'bias':
                a.evs_bias_file = str(self.pick('EVS bias file', [DEFAULT_BIAS] + bias_files(a.bias_root),
                                               a.evs_bias_file, lambda v: 'カメラ既定' if v == DEFAULT_BIAS else str(v)))
