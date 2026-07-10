#!/bin/bash
# Self-contained SONIC deploy launcher for a NON-interactive tmux pane (no ble.sh).
# Running this as the pane's direct command (tmux new-session -d -s X '<this>') avoids
# the ble.sh line editor that mangles send-keys, so the deploy's keyboard bring-up
# (] start, ENTER planner, 1 standing) is delivered reliably to the binary's stdin.
# Replicates the interactive env: conda g1_deploy + setup_env.sh, then the deploy.
cd /home/microagi/repos/GR00T-WholeBodyControl/gear_sonic_deploy
source scripts/setup_env.sh >/dev/null 2>&1
export CONTROL_WALL_SCALE=${CONTROL_WALL_SCALE:-0.333}   # match sim RTF
export PRED_HORIZON_S=${PRED_HORIZON_S:-0.0}
# AUTO_RECOVER defaults on in the binary (feed-loss -> damping -> auto re-arm).
exec ./target/release/g1_deploy_onnx_ref \
    lo policy/release/model_decoder.onnx reference/example/ \
    --disable-crc-check \
    --obs-config policy/release/observation_config.yaml \
    --encoder-file policy/release/model_encoder.onnx \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type manager --output-type all --zmq-host localhost
