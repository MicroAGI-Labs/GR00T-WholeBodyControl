#!/usr/bin/env bash
#
# RigPilot launcher — runs the Phoenix LiveView dashboard inside Docker.
#
# Phase 1: everything is a PLACEHOLDER. No rig process is spawned, no SSH, no
# DDS/ZMQ probing. The dashboard is driven by fake in-memory state only.
#
# Toolchain note: Elixir/Erlang are NOT installed on the rig host and there is
# no sudo, so we run the official `elixir` image (microagi is in the docker
# group). Files stay host-owned via --user; hex/deps/_build are cached on the
# host so repeat launches are fast.
#
# Serves on http://0.0.0.0:4000 (reachable on the rig LAN via --network host).
#
set -euo pipefail

IMAGE="elixir:1.18-otp-27"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Persistent mix cache (hex archives, phx_new). Shared with the build.
MIX_CACHE="${RIGPILOT_MIX_CACHE:-$HOME/.cache/rigpilot_mix}"

mkdir -p "$MIX_CACHE"

echo "RigPilot — launching dashboard (placeholder backends) on :4000"
echo "  image:    $IMAGE"
echo "  project:  $PROJECT_DIR"
echo "  mix cache: $MIX_CACHE"
echo

# Use a TTY only when launched interactively (backgrounded/CI has no TTY).
TTY_FLAGS=""
[ -t 0 ] && [ -t 1 ] && TTY_FLAGS="-it"

exec docker run --rm $TTY_FLAGS \
  --name rig_pilot \
  --network host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e MIX_HOME=/cache/.mix \
  -e MIX_ENV=dev \
  -e PHX_HOST=0.0.0.0 \
  -v "$MIX_CACHE":/cache \
  -v "$PROJECT_DIR":/app \
  -w /app \
  "$IMAGE" \
  sh -c 'mix deps.get && mix phx.server'
