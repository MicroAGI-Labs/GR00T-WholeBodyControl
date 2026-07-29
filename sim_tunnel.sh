#!/bin/bash
# ============================================================================
# Supervised DDS/camera tunnel to the RTX6000 Isaac-sim pod (Workstream A of
# SIM_RESILIENCE_PLAN.md).
#
# The pod is reachable ONLY over Cloudflare WARP on :22 (UDP blocked), so all
# DDS (:7447) and camera (:5555) traffic rides one SSH connection. A bare
# `ssh -N` is a single point of failure: it exits on any WARP hiccup / pod flap
# and nothing brings it back, so `rt/lowstate` silently stops and the G1 falls.
#
# autossh monitors the connection and re-establishes it automatically with
# backoff. Combined with the deploy's auto-recovery state (damping -> auto
# re-arm on feed return), a network flap heals with zero operator input.
#
#   Endpoint = the `rtx-pod` alias in ~/.ssh/config (single source of truth;
#   change the HostName there once after a pod restart).
#
# Usage:
#   ./sim_tunnel.sh              # foreground, supervised (Ctrl-C to stop)
#   ./sim_tunnel.sh &            # background
#   POD_SSH=rtx-pod ./sim_tunnel.sh   # override endpoint alias/host
# ============================================================================
set -u

POD_SSH="${POD_SSH:-rtx-pod}"     # ~/.ssh/config alias (or user@host)
DDS_PORT="${DDS_PORT:-7447}"
CAM_PORT="${CAM_PORT:-5555}"
SSH_COMPRESSION="${SSH_COMPRESSION:-yes}"

case "$SSH_COMPRESSION" in
  yes|no) ;;
  *) echo "[sim_tunnel] SSH_COMPRESSION must be 'yes' or 'no'" >&2; exit 2 ;;
esac

# autossh tuning:
#  AUTOSSH_GATETIME=0  -> restart even if a session dies within the first 30 s
#                         (WARP can drop a just-opened tunnel; we still want retry)
#  -M 0                -> no separate monitoring port; rely on SSH keepalive
#                         (ServerAliveInterval/CountMax from the rtx-pod alias)
export AUTOSSH_GATETIME=0
export AUTOSSH_POLL="${AUTOSSH_POLL:-30}"
export AUTOSSH_DEBUG="${AUTOSSH_DEBUG:-0}"

echo "[sim_tunnel] supervising DDS :$DDS_PORT and camera :$CAM_PORT to '$POD_SSH'"
echo "[sim_tunnel] autossh will auto-reconnect on any drop (keepalive ~15 s detect)"
echo "[sim_tunnel] SSH compression: $SSH_COMPRESSION"

# exec so signals (Ctrl-C / SIGTERM from a supervisor) go straight to autossh,
# which tears down the child ssh cleanly.
exec autossh -M 0 -N \
    -o "Compression=${SSH_COMPRESSION}" \
    -o ServerAliveInterval=5 \
    -o ServerAliveCountMax=3 \
    -o ExitOnForwardFailure=yes \
    -o StrictHostKeyChecking=accept-new \
    -L "${DDS_PORT}:localhost:${DDS_PORT}" \
    -L "${CAM_PORT}:localhost:${CAM_PORT}" \
    "$POD_SSH"
