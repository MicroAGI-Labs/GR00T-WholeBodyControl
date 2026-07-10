#!/bin/bash
# Reliable sim launcher: kills prior, daemonizes, PID file + known log.
cd ~/live-sim/unitree_sim_isaaclab
source ~/live-sim/venv/bin/activate
export LD_LIBRARY_PATH=~/live-sim/cyclonedds/install/lib:$LD_LIBRARY_PATH
pkill -9 -f "sim_main.py --device" 2>/dev/null; sleep 2
LOG=~/live-sim/sim_run.log
nohup python -u sim_main.py "$@" > "$LOG" 2>&1 &
echo $! > ~/live-sim/sim.pid
sleep 3
if kill -0 $(cat ~/live-sim/sim.pid) 2>/dev/null; then echo "STARTED pid=$(cat ~/live-sim/sim.pid) log=$LOG"; else echo "FAILED_TO_START"; tail -5 "$LOG"; fi
