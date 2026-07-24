#!/usr/bin/env bash
# Quick post-install checks for Haval H6 PHEV MK4 branch.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
echo "== openpilot =="
git -C "$ROOT" log -1 --oneline
echo "== opendbc_repo (expect 2e8e92ff or later on gwm-haval-h6-mk4-sp) =="
git -C "$ROOT/opendbc_repo" log -1 --oneline
echo "== GWM car interface =="
ls "$ROOT/opendbc_repo/opendbc/car/gwm"/{interface,carcontroller,carstate,gwmcan,values}.py
echo "== GWM safety =="
test -f "$ROOT/opendbc_repo/opendbc/safety/modes/gwm.h" && echo "gwm.h OK"
echo "== torqued allows gwm =="
grep -n "gwm" "$ROOT/openpilot/selfdrive/locationd/torqued.py" || true
echo "== done =="
