#!/bin/bash
# Self-contained SONIC deploy launcher for a NON-interactive tmux pane (no ble.sh).
# Running this as the pane's direct command (tmux new-session -d -s X '<this>') avoids
# the ble.sh line editor that mangles send-keys, so the deploy's keyboard bring-up
# (] start, ENTER planner, 1 standing) is delivered reliably to the binary's stdin.
# Replicates the interactive env: conda g1_deploy + setup_env.sh, then the deploy.
cd /home/microagi/repos/GR00T-WholeBodyControl/gear_sonic_deploy
source scripts/setup_env.sh >/dev/null 2>&1
export CONTROL_WALL_SCALE=${CONTROL_WALL_SCALE:-1.0}     # set to sim RTF; 1.0 on hardware
export PRED_HORIZON_S=${PRED_HORIZON_S:-0.0}
instance_id=${SONIC_INSTANCE_ID:-default}
DDS_INTERFACE=${DDS_INTERFACE:-$(ip -4 route show default | awk 'NR==1 {print $5}')}
DDS_INTERFACE=${DDS_INTERFACE:-lo}
# Namespaced controllers are the concurrent Spark simulation path.  Use an
# explicit CycloneDDS participant range there: the SDK default permits only
# indices 0..9, which is exhausted by eight controllers plus the local bridge.
if [[ -n "${SONIC_TOPIC_PREFIX:-}" && -z "${DDS_CONFIG_FILE:-}" ]]; then
  export DDS_CONFIG_FILE="$PWD/dds_sim_spark.json"
fi
if [[ -n "${SONIC_TOPIC_PREFIX:-}" ]]; then
  export SONIC_IDLE_REFERENCE_FILE="${SONIC_IDLE_REFERENCE_FILE:-/tmp/sonic_idle_reference_${instance_id}.txt}"
fi
case "$CONTROL_WALL_SCALE" in
  1|1.0|1.00|1.000) ;;
  *)
    export SONIC_IDLE_HOLD_REFERENCE=${SONIC_IDLE_HOLD_REFERENCE:-1}
    export SONIC_IDLE_PITCH_BIAS_DEG=${SONIC_IDLE_PITCH_BIAS_DEG:--8}
    ;;
esac
telemetry_args=()
if [[ -n "${IDLE_TELEMETRY_LOGFILE:-}" ]]; then
  telemetry_args=(--idle-telemetry-logfile "$IDLE_TELEMETRY_LOGFILE")
fi
zmq_input_port=${ZMQ_INPUT_PORT:-5556}
zmq_output_port=${ZMQ_OUTPUT_PORT:-5557}
zmq_input_topic=${ZMQ_INPUT_TOPIC:-pose_${instance_id}}
zmq_output_topic=${ZMQ_OUTPUT_TOPIC:-g1_debug_${instance_id}}
log_args=()
if [[ -n "${SONIC_LOGS_DIR:-}" ]]; then
  log_args=(--logs-dir "$SONIC_LOGS_DIR")
fi
# AUTO_RECOVER defaults on in the binary (feed-loss -> damping -> auto re-arm).
exec ./target/release/g1_deploy_onnx_ref \
    "$DDS_INTERFACE" policy/release/model_decoder.onnx reference/example/ \
    --disable-crc-check \
    --obs-config policy/release/observation_config.yaml \
    --encoder-file policy/release/model_encoder.onnx \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type manager --output-type all --zmq-host localhost \
    --zmq-port "$zmq_input_port" --zmq-topic "$zmq_input_topic" \
    --zmq-out-port "$zmq_output_port" --zmq-out-topic "$zmq_output_topic" \
    "${telemetry_args[@]}" "${log_args[@]}"
