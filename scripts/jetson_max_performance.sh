#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/jetson_max_performance.sh
  ./scripts/jetson_max_performance.sh --check

MAXN / MAXN_SUPER が選択済みであることを確認し、CPU・GPU・EMCを
最大クロックへ固定してファンを最大化します。

Options:
  --check  設定を変更せず、現在の電力モード、クロック、ファンを検証します。
  -h, --help
           このヘルプを表示します。

このスクリプトはJetsonホストで実行します。nvpmodelのモードは変更しません。
先に /etc/nvpmodel.conf でMAXN / MAXN_SUPERのIDを確認し、
`sudo nvpmodel -m ID`を実行してください。再起動を要求された場合は再起動します。
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    command -v sudo >/dev/null 2>&1 || die "sudoが見つかりません。rootで実行してください。"
    sudo "$@"
  fi
}

mode=apply
if (($# > 1)); then
  usage >&2
  exit 2
fi
if (($# == 1)); then
  case "$1" in
    --check) mode=check ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "不明なオプションです: $1" ;;
  esac
fi

[[ "$(uname -s)" == Linux && "$(uname -m)" == aarch64 ]] ||
  die "Jetsonホスト（Linux aarch64）で実行してください。"
[[ ! -e /.dockerenv && ! -e /run/.containerenv ]] ||
  die "コンテナではなくJetsonホストで実行してください。"

command -v nvpmodel >/dev/null 2>&1 || die "nvpmodelが見つかりません。Jetson上で実行してください。"
command -v jetson_clocks >/dev/null 2>&1 || die "jetson_clocksが見つかりません。Jetson上で実行してください。"

power_status="$(run_as_root nvpmodel -q 2>&1)"
echo "$power_status"
power_name="$(sed -n 's/^[[:space:]]*NV Power Mode:[[:space:]]*//p' <<< "$power_status" | head -n 1)"
case "$power_name" in
  MAXN|MAXN_SUPER) ;;
  *)
    echo "現在の電力モードは ${power_name:-取得できません} です。" >&2
    die "先に /etc/nvpmodel.conf のMAXN / MAXN_SUPERのIDで nvpmodel -m ID を実行し、必要なら再起動してください。"
    ;;
esac

if [[ "$mode" == apply ]]; then
  echo "${power_name}を確認しました。最大クロックと最大ファンを適用します。"
  run_as_root jetson_clocks --fan
  sleep 2
fi

clock_status="$(run_as_root jetson_clocks --show 2>&1)"
echo "$clock_status"

clock_domains=0
clock_failures=0
while IFS= read -r line; do
  case "$line" in
    cpu[0-9]*:*|GPU\ MinFreq=*|EMC\ MinFreq=*)
      if [[ "$line" =~ MinFreq=([0-9]+).*MaxFreq=([0-9]+) ]]; then
        clock_domains=$((clock_domains + 1))
        if [[ "${BASH_REMATCH[1]}" != "${BASH_REMATCH[2]}" ]]; then
          echo "Clock is not fixed: $line" >&2
          clock_failures=$((clock_failures + 1))
        fi
      fi
      ;;
  esac
done <<< "$clock_status"

((clock_domains >= 3)) || die "CPU・GPU・EMCのクロック情報を十分に取得できませんでした。"
((clock_failures == 0)) || die "最大クロックに固定されていないdomainがあります。"

grep -q 'FAN Dynamic Speed Control=disabled.*pwm1=255' <<< "$clock_status" ||
  die "ファンが最大PWM 255に固定されていません。"

if [[ "$mode" == check ]]; then
  echo "OK: ${power_name}、最大クロック、最大ファンを確認しました。"
else
  echo "OK: ${power_name}で最大クロックと最大ファンを適用・確認しました。"
fi
