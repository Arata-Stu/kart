#!/usr/bin/env python3
"""Interactive STOP-only, atomic fixed-command configuration (ROS Python)."""
import argparse
import math

FIELDS = ('fixed_axes', 'steering', 'throttle', 'brake', 'duration_s')


def trial_values(current, choice, ask=input):
    """Build the entire transaction before contacting the running controller."""
    axes = {'1': 'throttle', '2': 'steering', '3': 'both'}[choice]
    values = dict(current, fixed_axes=axes)
    keys = (['steering'] if axes == 'steering' else
            ['throttle', 'brake'] if axes == 'throttle' else
            ['steering', 'throttle', 'brake'])
    for key in keys + ['duration_s']:
        text = ask(f'{key} [{current[key]}]: ').strip()
        values[key] = float(text) if text else current[key]
    if any(not math.isfinite(values[k]) for k in FIELDS[1:]):
        raise ValueError('数値は有限値にしてください')
    if (abs(values['steering']) > 1 or not 0 <= values['throttle'] <= current['max_throttle'] or
            not 0 <= values['brake'] <= 1 or values['duration_s'] < 0 or
            (values['throttle'] > 0 and values['brake'] > 0)):
        raise ValueError('範囲外、またはthrottleとbrakeが同時に正です')
    return {k: values[k] for k in FIELDS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', default='/fixed_command')
    args = parser.parse_args()
    import rclpy
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient
    rclpy.init()
    node = rclpy.create_node('vehicle_tuning_control')
    client = AsyncParameterClient(node, args.node)

    def wait(future):
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if not future.done():
            raise RuntimeError('応答未確認です。STOPを維持し、再接続して現在値を確認してください')
        return future.result()

    try:
        if not client.wait_for_services(timeout_sec=3.0):
            raise RuntimeError(f'{args.node} のparameter serviceがありません')
        print('変更前に車両をSTOPにしてください。録画とAUTO操作はコントローラで行います。')
        print('q/Ctrl-Cは画面のみ終了します。車両の停止操作ではありません。')
        while True:
            names = (*FIELDS, 'max_throttle')
            response = wait(client.get_parameters(list(names)))
            current = dict(zip(names, map(parameter_value_to_python, response.values)))
            print('\n現在の設定: ' + ', '.join(f'{k}={v}' for k, v in current.items()))
            choice = input('1:スロットル固定  2:ステア固定  3:両方固定  r:再読込  q:終了 > ').strip()
            if choice == 'q':
                break
            if choice not in ('1', '2', '3'):
                continue
            try:
                values = trial_values(current, choice)
            except ValueError as exc:
                print(f'未適用: {exc}')
                continue
            response = wait(client.set_parameters_atomically(
                [Parameter(k, value=v) for k, v in values.items()]))
            if response.result.successful:
                print('設定受付済み。次のAUTOで使用します（次画面で現在値を確認）。')
            else:
                print(f'未適用: {response.result.reason}')
    except (KeyboardInterrupt, EOFError):
        print('\n画面を終了しました。車両モードは変更していません。')
    except RuntimeError as exc:
        print(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
