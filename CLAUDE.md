# CLAUDE.md — GR00T-WholeBodyControl working notes

## RTX6000 pod: SSH + sshfs (edit pod files directly)

The Isaac-sim stack lives on the **RTX6000 pod**, reached over Cloudflare WARP (port 22
only, UDP blocked). SSH alias: **`rtx-pod`** (`ssh rtx-pod`), defined in `~/.ssh/config`.

- **The pod's IP is the single source of truth in `~/.ssh/config` → `Host rtx-pod` → `HostName`.**
  After a pod restart the WARP IP changes (e.g. `10.5.7.178`). Update `HostName` there in
  **one place** and every consumer (`sim_tunnel.sh`, sshfs mount, all `ssh rtx-pod` calls)
  picks it up. Do NOT hardcode the IP anywhere else.

- **The pod's `~/live-sim` is sshfs-mounted on the Spark at `/home/microagi/pod-live-sim`.**
  Edit pod-side files (USDs via pxr scripts, env cfgs, `*.py`, `*.sh`) directly through this
  mount with normal Read/Write/Edit — no scp needed. It maps to
  `spark@<pod-ip>:/home/spark/live-sim`. Note `~/live-sim` is a **separate checkout**, not
  part of this repo (see `rtx_pod/README.md`).

- **Check the mount first:** `mount | grep sshfs` — expect
  `spark@<ip>:/home/spark/live-sim on /home/microagi/pod-live-sim type fuse.sshfs`.

- **If the mount is missing / stale** (common after a pod IP change — the old mount goes
  dead and Read/Write hang or error): remount with the *current* `rtx-pod` alias so the IP
  stays consistent:
  ```bash
  fusermount -u /home/microagi/pod-live-sim 2>/dev/null   # clear a dead/stale mount
  sshfs rtx-pod:/home/spark/live-sim /home/microagi/pod-live-sim \
        -o reconnect,ServerAliveInterval=5,ServerAliveCountMax=3
  ```
  (`sshfs` is at `/usr/bin/sshfs`. Using the `rtx-pod` alias — not a literal IP — means the
  mount tracks whatever `HostName` currently is.)

- **DDS/camera tunnel** (separate from the mount): `./sim_tunnel.sh` runs autossh forwarding
  DDS `:7447` and camera `:5555` over the same `rtx-pod` endpoint, auto-reconnecting on WARP
  flaps. It also reads the endpoint from the `rtx-pod` alias.

## tmux-mcp: for anything that needs real keystrokes to a PTY

Use the **tmux MCP tools** (`mcp__tmux__*`: `list-sessions`, `list-panes`, `capture-pane`,
`send-keys` via `execute-command`, `create-session`, etc.) for interactive processes that
must receive **live keystrokes on a real terminal**, not just a launched command line.

- **Why a PTY matters:** the SONIC deploy (`g1_deploy_onnx_ref`) keyboard handler uses
  **termios raw mode** — it reads single keypresses (`]` enable planner, `1` stand, etc.)
  from a real terminal. A pipe/FIFO/redirected stdin does **not** work; it needs a PTY,
  which a tmux pane provides.

- **Footgun — login-shell line editors eat send-keys:** panes running an interactive login
  shell with `ble.sh` mangle/swallow long `send-keys` strings and wedge after Ctrl-C. Run the
  target as the pane's **direct command** (no login shell) so keystrokes reach its PTY
  cleanly:
  ```bash
  # launch the interactive CLI as the pane's direct command (no login shell):
  tmux new-session -d -s dep "<launch cmd> > /tmp/dep.log 2>&1"
  # then send single keys to its PTY:
  tmux send-keys -t dep ']' Enter '1'
  ```
  (For the flat-balance workflow the loop is now engaged over ZMQ instead of keystrokes —
  see `drive_stand.py` — but keystroke control still applies to any deploy/CLI that reads a
  raw terminal.)

- **Inspecting the running stack:** enumerate every session/window/pane before assuming
  state — `mcp__tmux__list-sessions` then `capture-pane` each relevant pane (deploy, driver,
  tunnel, warmer). The plain `tmux` CLI via Bash also works for scripted `send-keys`; prefer
  the MCP tools when you need structured session/pane discovery and capture.
