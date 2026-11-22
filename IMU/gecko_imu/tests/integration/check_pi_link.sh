#!/usr/bin/env bash
# Check the Raspberry helpers without a Raspberry: the remote scripts run locally
# against a fake sender, and the SSH search is pointed at a host that cannot answer.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TMP="$(mktemp -d)"
FAKE="$ROOT/tests/fixtures/fake_pi/bno085_sender.py"
PATTERN='^[^ ]*python[^ ]* .*[b]no085_sender\.py'
export GECKO_SENDER_LOG="$TMP/sender.log"

cleanup() { pkill -KILL -f "$PATTERN" 2>/dev/null; rm -rf "$TMP"; }
trap cleanup EXIT
die() { echo "FAIL: $*" >&2; exit 1; }

start_remote() {
    bash "$ROOT/scripts/pi_remote/start_sender.sh" /usr/bin/python3 "$FAKE" --host 10.42.0.1 --port 5005 --rate 30
}

out="$(start_remote)" || die "start failed: $out"
grep -q '^running' <<<"$out" || die "start did not report running: $out"
[[ "$(pgrep -fc "$PATTERN")" == 1 ]] || die 'expected one sender after start'
grep -q 'args: --host 10.42.0.1 --port 5005 --rate 30' "$GECKO_SENDER_LOG" || die 'arguments not passed'
first="$(pgrep -f "$PATTERN")"

out="$(start_remote)" || die "second start failed: $out"
[[ "$(pgrep -fc "$PATTERN")" == 1 ]] || die 'a second start left two senders'
[[ "$(pgrep -f "$PATTERN")" != "$first" ]] || die 'the old sender was not replaced'

out="$(bash "$ROOT/scripts/pi_remote/stop_sender.sh")" || die "stop failed: $out"
sleep 0.5
pgrep -f "$PATTERN" >/dev/null && die 'sender still running after stop'
grep -q 'BNO085 sender stopped.' "$GECKO_SENDER_LOG" || die 'sender was not stopped with Ctrl+C'
bash "$ROOT/scripts/pi_remote/stop_sender.sh" >/dev/null || die 'second stop failed'

out="$(bash "$ROOT/scripts/pi_remote/start_sender.sh" /usr/bin/python3 "$TMP/missing.py" 2>&1)"
[[ $? == 2 ]] || die 'a missing sender should exit with code 2'
grep -q 'missing.py' <<<"$out" || die "missing sender not reported: $out"

begin=$SECONDS
out="$(PI_HOST=127.0.0.1 PI_WAIT_S=4 WIFI_IFACE=lo STATE_DIR="$TMP/state" \
       bash "$ROOT/scripts/gecko_pi.sh" start 2>&1)"
code=$?
[[ $code != 0 ]] || die 'start reported success without a Raspberry'
grep -q 'No se encontró la Raspberry' <<<"$out" || die "unclear message: $out"
(( SECONDS - begin <= 20 )) || die 'search did not respect PI_WAIT_S'

out="$(PI_HOST=127.0.0.1 WIFI_IFACE=lo STATE_DIR="$TMP/state" bash "$ROOT/scripts/gecko_pi.sh" stop 2>&1)" ||
    die "stop without a Raspberry should not fail: $out"

out="$(PI_HOST=127.0.0.1 PI_WAIT_S=3 WIFI_IFACE=lo STATE_DIR="$TMP/state" "$ROOT/start_gecko_pc.sh" pi 2>&1)"
[[ $? != 0 ]] || die 'start_gecko_pc.sh pi reported success without a Raspberry'
grep -q 'No se encontró la Raspberry' <<<"$out" || die "start_gecko_pc.sh pi message: $out"

echo pi_link_ok
