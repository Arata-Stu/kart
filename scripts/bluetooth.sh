#!/usr/bin/env bash
# DualSense setup via host BlueZ, from Linux host or the kart development container.
set -euo pipefail
export LC_ALL=C
# Source: JetPilot/scripts/bluetooth.sh SECOND_CONTROLLER_MAC (PS5 DualSense).
kart_controller_mac=${KART_PS5_MAC:-4C:B9:9B:E0:EF:24}

usage() {
  cat <<'HELP'
Usage: scripts/bluetooth.sh [--scan | --pair [MAC] | --connect [MAC] | --status [MAC]]

Linuxホスト／kart DockerからホストのBlueZを操作するDualSense接続用ツール。
Dockerではホストの/run/dbus共有とbluez導入が必要（kart Docker設定に含む）。
  引数なし       bluetoothctlを対話モードで開く
  --scan         15秒間デバイスを検索する
  --pair [MAC]    ペアリング・trust・接続（各操作最大30秒）
  --connect [MAC] 登録済みデバイスへ再接続
  --status [MAC]  デバイス情報を表示
  -h, --help     このヘルプを表示

MAC省略時: 4C:B9:9B:E0:EF:24（JetPilotのPS5 DualSense）
KART_PS5_MACで既定MACを変更可能。明示したMACを優先する。

初回はDualSenseのCreate＋PSを長押しし、ライトを点滅させる。
登録済みPS5は--pair、次回から--connectで接続する。
別のコントローラは--scanで確認し、--pair AA:BB:CC:DD:EE:FFを指定する。
HELP
}

kart_mode=${1:-interactive}
case "$kart_mode" in
  -h|--help) usage; exit 0 ;;
  interactive|--scan) [[ $# -le 1 ]] || { usage >&2; exit 2; } ;;
  --pair|--connect|--status)
    kart_controller_mac=${2:-$kart_controller_mac}
    if [[ $# -gt 2 || ! "$kart_controller_mac" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]]; then
      usage >&2; exit 2
    fi
    ;;
  *) usage >&2; exit 2 ;;
esac

if [[ "$(uname -s)" != Linux ]]; then
  echo 'JetsonまたはUbuntu PC上のLinuxホスト／kart Dockerで実行してください。' >&2
  exit 1
fi
if [[ -f /.dockerenv || -f /run/.containerenv ]]; then
  if [[ ! -S /run/dbus/system_bus_socket ]]; then
    echo 'ホストのD-Bus socketがありません。更新したdocker/dockerargsでコンテナを作り直してください。' >&2
    exit 1
  fi
  export DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket
fi
if ! command -v bluetoothctl >/dev/null 2>&1; then
  echo 'bluetoothctlがありません。ホストはsudo apt install bluez、kart Dockerはimageを再buildしてください。' >&2
  exit 1
fi

case "$kart_mode" in
  interactive)
    cat <<'HELP'
DualSense: Create＋PS長押しでペアリングモード。
bluetoothctl内で次を順に実行してください:
  power on
  agent NoInputNoOutput
  default-agent
  scan on
  pair <DualSenseのMAC>
  trust <DualSenseのMAC>
  connect <DualSenseのMAC>
  scan off
  quit
HELP
    exec bluetoothctl
    ;;
  --scan)
    bluetoothctl --timeout 10 power on
    exec bluetoothctl --timeout 15 scan on
    ;;
  --status) exec bluetoothctl --timeout 10 info "$kart_controller_mac" ;;
  --pair|--connect)
    printf "DualSense: %s\n" "$kart_controller_mac"
    bluetoothctl --timeout 10 power on
    if [[ "$kart_mode" == --pair ]]; then
      bluetoothctl --agent NoInputNoOutput --timeout 30 pair "$kart_controller_mac"
      bluetoothctl --timeout 10 trust "$kart_controller_mac"
    fi
    bluetoothctl --timeout 30 connect "$kart_controller_mac"
    kart_info=$(bluetoothctl --timeout 10 info "$kart_controller_mac")
    printf '%s\n' "$kart_info"
    if [[ "$kart_info" != *'Connected: yes'* ]]; then
      echo '接続を確認できません。PSボタン・ホストのBluetoothサービス・rfkill状態を確認してください。' >&2
      exit 1
    fi
    ;;
esac
