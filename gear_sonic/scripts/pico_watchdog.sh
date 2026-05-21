#!/usr/bin/env bash
# ============================================================================
# pico_watchdog.sh — supervise roboticsservice + pico_manager for PICO teleop.
#
# Restarts the PICO pipeline when it either:
#   (a) DIES        — pico_manager process is gone, or
#   (b) WEDGES      — process is up but /tmp/pico_manager.log has stopped
#                     growing (the "device missing" stall after a PICO/network
#                     drop; healthy teleop writes pose debug every ~0.5s).
#
# A restart kills both pico_manager and RoboticsServiceProcess, then relaunches
# pico_manager (which starts a fresh roboticsservice via runService.sh). A
# cooldown avoids thrashing while the PICO is genuinely offline.
#
# Run:   nohup gear_sonic/scripts/pico_watchdog.sh >/tmp/pico_watchdog.log 2>&1 &
# Stop:  pkill -f pico_watchdog.sh
# ============================================================================
set -u

REPO="${REPO:-/home/microagi/repos/GR00T-WholeBodyControl}"
PY="${PICO_PY:-$REPO/.venv_teleop/bin/python}"
MANAGER_ARGS="${PICO_MANAGER_ARGS:---manager}"   # no --vis_rerun (keeps CPU down)
MANAGER_LOG=/tmp/pico_manager.log

INTERVAL="${WD_INTERVAL:-15}"   # seconds between checks
STALE="${WD_STALE:-20}"         # log-age (s) that counts as wedged
GRACE="${WD_GRACE:-35}"         # seconds after a restart before re-checking
COOLDOWN="${WD_COOLDOWN:-90}"   # min seconds between restarts

last_restart=0
ts(){ date '+%F %T'; }

log_age(){ # seconds since pico_manager.log last changed; huge if missing
  if [ -f "$MANAGER_LOG" ]; then echo $(( $(date +%s) - $(stat -c %Y "$MANAGER_LOG") ));
  else echo 999999; fi
}

restart_pipeline(){
  local reason="$1" now; now=$(date +%s)
  if (( now - last_restart < COOLDOWN )); then
    echo "$(ts) [wd] $reason — in cooldown ($(( COOLDOWN - (now - last_restart) ))s left), skip"
    return
  fi
  last_restart=$now
  echo "$(ts) [wd] $reason — restarting roboticsservice + pico_manager"
  pkill -f pico_manager_thread_server.py 2>/dev/null || true
  pkill -f RoboticsServiceProcess 2>/dev/null || true
  sleep 3
  ( cd "$REPO" && nohup "$PY" gear_sonic/scripts/pico_manager_thread_server.py $MANAGER_ARGS >"$MANAGER_LOG" 2>&1 & )
  echo "$(ts) [wd] pico_manager relaunched; ${GRACE}s grace"
  sleep "$GRACE"
}

echo "$(ts) [wd] watchdog started (interval=${INTERVAL}s stale=${STALE}s cooldown=${COOLDOWN}s)"
while true; do
  if ! pgrep -f pico_manager_thread_server.py >/dev/null 2>&1; then
    restart_pipeline "pico_manager DOWN"
  elif [ "$(log_age)" -gt "$STALE" ]; then
    restart_pipeline "pico_manager WEDGED (log idle $(log_age)s)"
  fi
  sleep "$INTERVAL"
done
