# RigPilot

An Elixir/Phoenix **LiveView control-center dashboard** for the DGX Spark +
Unitree G1 teleop rig. See the full design in
[`../ELIXIR_SUPERVISOR_GOAL.md`](../ELIXIR_SUPERVISOR_GOAL.md).

> **Phase 1 — placeholders only.** This build implements the *UI* of Phase 1.
> **Nothing here touches the live rig.** No subprocess is spawned, no SSH, no
> DDS/ZMQ/PICO probing, no actuation. Every process card and every telemetry
> number is fake, in-memory state. It is a polished, interactive UI you can
> click around safely.

## What's here

- **Pipeline graph** across the top: `PICO → roboticsservice → pico_manager →
  ZMQ → deploy → zenoh → Orin → robot`, each hop green/amber/red derived from
  the placeholder health.
- **Per-process cards** for `roboticsservice`, `pico_manager`,
  `zenoh_spark_bridge`, `orin_bridge`, `deploy`, `sim`: status badge, host,
  uptime, restart count, recent (fake) log lines, and working
  **Start · Stop · Restart** buttons. The `deploy` Start is gated behind a
  confirm modal and shows its `--output-type` (warns loudly if `log` = dry-run).
- **Telemetry panel**: numeric tiles + inline SVG sparklines for `lowstate_hz`,
  `lowcmd_hz`, PICO `fps`; a PICO body badge **LIVE/STALE keyed on `pose_chg`**
  (never `body_ts` — that flag is a known PICO-app bug and is ignored); zenoh
  sessions; robot pings; secondary IMU; deploy output type.
- A prominent always-visible **E-STOP** (placeholder: stops the deploy card and
  flashes a banner).

State flows over `Phoenix.PubSub` topic `"rig:status"`:
`RigPilot.TelemetrySource` broadcasts `{:telemetry, snapshot}` ~1 Hz;
`RigPilot.ManagedProc` broadcasts `:procs_changed` on lifecycle events;
`RigPilotWeb.DashboardLive` subscribes and re-renders.

## Run it

Elixir/Erlang are not installed on the rig host (and there's no sudo), so
RigPilot runs in Docker. The `microagi` user is in the docker group.

```bash
./run.sh
```

Then open **http://localhost:4000** (or `http://<spark-lan-ip>:4000` from
another machine — the container uses `--network host` and binds `0.0.0.0:4000`).

`run.sh` uses the official `elixir:1.18-otp-27` image, keeps files host-owned
(`--user "$(id -u):$(id -g)"`), and caches hex + deps + `_build` under
`~/.cache/rigpilot_mix` so repeat launches are fast. Stop with Ctrl-C.

## Layout

```
rig_pilot/
├── run.sh                            # Docker launcher (port 4000, host net)
├── README.md
├── mix.exs
├── config/                          # default phx.new config (dev binds 0.0.0.0:4000)
├── lib/
│   ├── rig_pilot/
│   │   ├── application.ex           # supervision tree (Registry, ProcSupervisor, TelemetrySource)
│   │   ├── managed_proc.ex          # PLACEHOLDER GenServer, one per rig subprocess
│   │   ├── proc_supervisor.ex       # supervises the ManagedProc children
│   │   └── telemetry_source.ex      # PLACEHOLDER fake telemetry (~1 Hz PubSub)
│   └── rig_pilot_web/
│       ├── live/dashboard_live.ex   # the dashboard LiveView
│       ├── router.ex                # "/" → DashboardLive
│       └── components/layouts/      # full-bleed dark layout
└── assets/                          # default phx.new tailwind/esbuild
```

## Phase 2 — where the real wiring slots in

The placeholder modules each carry a `PLACEHOLDER — wire to muontrap/SSH in
Phase 2` doc comment marking the exact swap points. The PubSub contract and the
LiveView do **not** change between phases.

- **`lib/rig_pilot/managed_proc.ex`** — replace `do_start/1`, `do_stop/1`, and
  the `:finish_start` transition with real spawning:
  - Spark procs (`roboticsservice`, `pico_manager`, `zenoh_spark_bridge`,
    `deploy`, `sim`): `MuonTrap.Daemon.start_link/...` (cgroup-scoped →
    guaranteed child cleanup, fixes the zombie/`pkill` mess). Stream real
    stdout/stderr into `recent_logs`.
  - `orin_bridge`: `System.cmd("ssh", ["unitree@192.168.123.164",
    "~/zenoh-dds/start_bridge.sh"])` (or `:erlexec`).
  - Add a periodic `health_check/1` that flips `:wedged` using the §5 signals
    (e.g. log-idle > 20 s, `lowcmd_hz == 0` while running) instead of fake state.
- **`lib/rig_pilot/proc_supervisor.ex`** — swap the `:one_for_one` `Supervisor`
  for a `rest_for_one` / `DynamicSupervisor` with dependency ordering
  (`roboticsservice → pico_manager`,
  `orin_bridge → zenoh_spark_bridge → deploy`), exponential backoff and
  restart-storm protection (§8).
- **`lib/rig_pilot/telemetry_source.ex`** — replace `gen/1` with a `Port` on the
  long-lived Python telemetry sidecar (cyclonedds / `xrobotoolkit_sdk`), decode
  one JSON line per ~500 ms (the §6.3 contract this fake data already matches),
  and merge with `RigPilot.ManagedProc.all/0` in a HealthAggregator.
- **E-STOP** (`dashboard_live.ex` `handle_event("estop", ...)`) — make it
  actually cut `rt/lowcmd` (kill deploy) within one tick, the fastest path in
  the app.
- Add **basic auth** + bind to the rig LAN before exposing real controls.

Built with **Elixir 1.18.4 / Erlang/OTP 27**, **Phoenix 1.7.21**,
**Phoenix LiveView 1.1**.
