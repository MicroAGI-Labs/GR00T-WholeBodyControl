#!/bin/bash
# Run the SONIC deploy binary directly (no deploy.sh, no docker-bridge recreation)
# against the already-warm zenoh route. Rate-matched to Isaac RTF via CONTROL_WALL_SCALE.
#
# Against the RTX6000 pod sim: the ~38ms WARP round-trip caps the stable RTF. SONIC
# balances the G1 unaided up to RTF~0.333 (pair with pod /tmp/sim_slowmo=3). Above that
# the loop delay exceeds the policy's ~18-20ms delay margin and it falls (prediction and
# gain-soften don't extend it -- see memory sonic-rtf-delay-tolerance-levers).
#   CONTROL_WALL_SCALE MUST equal the sim RTF (=1/sim_slowmo).
# For the REAL robot: unset these knobs -> CONTROL_WALL_SCALE defaults to 1.0 and all
# sim-only IDLE trim/prediction settings remain off.
#   Override at launch, e.g.:  CONTROL_WALL_SCALE=0.333 ./run_deploy_direct.sh
cd /home/microagi/repos/GR00T-WholeBodyControl/gear_sonic_deploy
export CONTROL_WALL_SCALE=${CONTROL_WALL_SCALE:-1.0}     # set to sim RTF; 1.0 on the real robot
export PRED_HORIZON_S=${PRED_HORIZON_S:-0.0}             # forward state prediction; 0=off (see /tmp/pred_horizon to sweep live)
DDS_INTERFACE=${DDS_INTERFACE:-$(ip -4 route show default | awk 'NR==1 {print $5}')}
DDS_INTERFACE=${DDS_INTERFACE:-lo}
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
instance_id=${SONIC_INSTANCE_ID:-default}
zmq_input_port=${ZMQ_INPUT_PORT:-5556}
zmq_output_port=${ZMQ_OUTPUT_PORT:-5557}
zmq_input_topic=${ZMQ_INPUT_TOPIC:-pose_${instance_id}}
zmq_output_topic=${ZMQ_OUTPUT_TOPIC:-g1_debug_${instance_id}}
log_args=()
if [[ -n "${SONIC_LOGS_DIR:-}" ]]; then
  log_args=(--logs-dir "$SONIC_LOGS_DIR")
fi
exec ./target/release/g1_deploy_onnx_ref \
    "$DDS_INTERFACE" \
    policy/release/model_decoder.onnx \
    reference/example/ \
    --disable-crc-check \
    --obs-config policy/release/observation_config.yaml \
    --encoder-file policy/release/model_encoder.onnx \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type manager \
    --output-type all \
    --zmq-host localhost \
    --zmq-port "$zmq_input_port" \
    --zmq-topic "$zmq_input_topic" \
    --zmq-out-port "$zmq_output_port" \
    --zmq-out-topic "$zmq_output_topic" \
    "${telemetry_args[@]}" \
    "${log_args[@]}"
