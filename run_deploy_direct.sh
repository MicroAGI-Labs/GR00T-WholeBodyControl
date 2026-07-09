#!/bin/bash
# Run the SONIC deploy binary directly (no deploy.sh, no docker-bridge recreation)
# against the already-warm zenoh route. Rate-matched to Isaac RTF via CONTROL_WALL_SCALE.
#
# Against the RTX6000 pod sim: the ~38ms WARP round-trip caps the stable RTF. SONIC
# balances the G1 unaided up to RTF~0.333 (pair with pod /tmp/sim_slowmo=3). Above that
# the loop delay exceeds the policy's ~18-20ms delay margin and it falls (prediction and
# gain-soften don't extend it -- see memory sonic-rtf-delay-tolerance-levers).
#   CONTROL_WALL_SCALE MUST equal the sim RTF (=1/sim_slowmo).
# For the REAL robot: unset these knobs -> CONTROL_WALL_SCALE defaults to 1.0, PRED/gain
# scaling default to no-op, so behaviour is identical to before (interface unchanged).
#   Override at launch, e.g.:  CONTROL_WALL_SCALE=0.333 ./run_deploy_direct.sh
cd /home/microagi/repos/GR00T-WholeBodyControl/gear_sonic_deploy
export CONTROL_WALL_SCALE=${CONTROL_WALL_SCALE:-0.333}   # sim RTF; use 1.0 for the real robot
export PRED_HORIZON_S=${PRED_HORIZON_S:-0.0}             # forward state prediction; 0=off (see /tmp/pred_horizon to sweep live)
exec ./target/release/g1_deploy_onnx_ref \
    lo \
    policy/release/model_decoder.onnx \
    reference/example/ \
    --disable-crc-check \
    --obs-config policy/release/observation_config.yaml \
    --encoder-file policy/release/model_encoder.onnx \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type manager \
    --output-type all \
    --zmq-host localhost
