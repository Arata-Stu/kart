#!/usr/bin/env bash
# Run on the Jetson host, not inside the ROS container.
set -euo pipefail
export LC_ALL=C
iface=wlP1p1s0
uuid=d2c82d15-9fd1-4174-9f30-aac3c61e76c2
usage() {
  cat <<'EOF'
Usage: scripts/hotspot.sh [start|stop|status|diagnose]
  start    既存のHotspot-Internal-5GHzを保存済み設定で起動
  stop     対象APだけ停止（通常Wi-Fiには自動復帰しません）
  status   interface・接続・IPを表示
  diagnose AP対応・rfkill・NetworkManagerの直近ログを表示
固定interface: wlP1p1s0
既存UUID: d2c82d15-9fd1-4174-9f30-aac3c61e76c2
設定名に5GHzが残っていても現在の保存設定は2.4 GHz/channel 6。
新規作成・設定変更は行いません。自動起動はNetworkManagerの保存設定に従います。
Jetsonホストで実行してください。startで同じinterfaceのWi-Fi接続は切り替わります。
EOF
}
fail() { echo "ERROR: $*" >&2; exit 1; }
action=${1:-start}
[[ $# -le 1 ]] || { usage >&2; exit 2; }
case "$action" in -h|--help) usage; exit 0;; start|stop|status|diagnose) ;; *) usage >&2; exit 2;; esac
command -v nmcli >/dev/null || fail 'nmcliがありません。JetsonホストのNetworkManagerを確認してください。'
if [[ $action == start || $action == stop ]]; then
  [[ $EUID == 0 ]] || fail "sudo bash scripts/hotspot.sh $action で実行してください。"
fi
nmcli -t -f GENERAL.TYPE device show "$iface" >/dev/null || fail "$iface が見つかりません。nmcli device status を確認してください。"
status() {
  nmcli -f connection.id,connection.autoconnect,802-11-wireless.band,802-11-wireless.channel connection show uuid "$uuid"
  nmcli -f GENERAL.DEVICE,GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS device show "$iface"
}
if [[ $action == status ]]; then status; exit; fi
if [[ $action == diagnose ]]; then
  status
  nmcli -f WIFI-PROPERTIES device show "$iface"
  nmcli radio
  if command -v rfkill >/dev/null; then rfkill list; fi
  if command -v iw >/dev/null; then
    iw reg get
    iw dev "$iface" info
    phy=$(basename "$(readlink -f "/sys/class/net/$iface/phy80211")")
    iw phy "$phy" info
  fi
  if command -v journalctl >/dev/null; then journalctl -u NetworkManager -n 80 --no-pager; fi
  exit
fi
# Use the known existing profile, regardless of subsequent display-name changes.
nmcli -g connection.uuid connection show uuid "$uuid" >/dev/null 2>&1 || fail '既存hotspotのUUIDがありません。nmcli connection showで確認してください。新規作成は行いません。'
[[ $(nmcli -g connection.interface-name connection show uuid "$uuid") == "$iface" &&
   $(nmcli -g 802-11-wireless.mode connection show uuid "$uuid") == ap ]] || fail '対象profileのinterfaceまたはmodeが想定外です。変更せず中止しました。'
if [[ $action == stop ]]; then
  if [[ -n $uuid ]] && nmcli -g UUID connection show --active | grep -Fxq "$uuid"; then
    nmcli --wait 15 connection down uuid "$uuid"
  else
    echo 'kart hotspotは停止済みです。'
  fi
  exit
fi
[[ $(nmcli -g WIFI-PROPERTIES.AP device show "$iface") == yes ]] || fail 'AP機能が利用できません。diagnoseでドライバ・adapterを確認してください。'
echo "$iface をhotspotへ切り替えます。同じWi-Fi経由のSSHは切断されます。"
if ! nmcli --wait 30 connection up uuid "$uuid" ifname "$iface"; then
  echo '起動失敗。sudo bash scripts/hotspot.sh diagnose で確認してください。保存profileは残っています。' >&2
  exit 1
fi
status
echo '表示されたIPv4アドレスがJetsonの接続先です。インターネットなしのWi-Fiでもローカル通信は可能です。'
