#!/bin/bash
# sim_watchdog.sh — pod-side liveness watchdog (SIM_RESILIENCE_PLAN.md Workstream D).
#
# The pod analog of autossh: autossh heals the tunnel and the deploy auto-recovers,
# but nothing restarts the SIM ITSELF if its process dies or wedges (stops publishing
# rt/lowstate). This watchdog closes that gap: if rt/lowstate is absent for a grace
# window, it restarts the whole stack via the hardened start_flat.sh (which is
# single-instance + robust-kill). It NEVER `rm`s /dev/shm (that would break the
# self-healing shm re-attach, WS-C).
#
# Usage:  SIM_BASE_HOLD_S=5 SIM_BASE_SOFT=1 nohup ~/live-sim/sim_watchdog.sh &
set -uo pipefail
LS="$HOME/live-sim"
GRACE="${WATCHDOG_GRACE_S:-20}"     # restart if lowstate absent this long
POLL="${WATCHDOG_POLL_S:-5}"        # check cadence
export SIM_BASE_HOLD_S="${SIM_BASE_HOLD_S:-5}"
export SIM_BASE_SOFT="${SIM_BASE_SOFT:-1}"

# returns 0 if >=1 rt/lowstate sample arrives on DDS domain 1 within 2s
lowstate_alive() {
  source "$LS/venv/bin/activate" 2>/dev/null
  LD_LIBRARY_PATH="$LS/cyclonedds/install/lib:/usr/local/nvidia/lib64" \
  env -u CYCLONEDDS_URI python - <<'PY' 2>/dev/null
import sys, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
ChannelFactoryInitialize(1)
n=[0]
s=ChannelSubscriber("rt/lowstate", LowState_); s.Init(lambda m: n.__setitem__(0, n[0]+1), 5)
time.sleep(2.0)
sys.exit(0 if n[0] > 0 else 1)
PY
}

echo "[watchdog] started (grace=${GRACE}s poll=${POLL}s, hold=${SIM_BASE_HOLD_S} soft=${SIM_BASE_SOFT})"
absent=0
while true; do
  if lowstate_alive; then
    absent=0
  else
    absent=$((absent + POLL + 2))
    echo "[watchdog] rt/lowstate ABSENT for ~${absent}s (grace ${GRACE}s)"
    if [ "$absent" -ge "$GRACE" ]; then
      echo "[watchdog] $(date -Is) restarting sim stack via start_flat.sh (NO /dev/shm wipe)"
      bash "$LS/start_flat.sh" >> "$LS/watchdog_restart.log" 2>&1
      absent=0
      sleep 10   # let it settle before polling again
    fi
  fi
  sleep "$POLL"
done
