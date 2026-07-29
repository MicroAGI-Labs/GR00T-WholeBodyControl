# Elixir Rig Supervisor — Goal & Design

**Working name:** `RigPilot` (an Elixir/OTP app that supervises every subprocess in the
DGX Spark + Unitree G1 teleop rig and presents a polished real-time Phoenix LiveView
dashboard with safe start/stop/restart control.)

> Status: design/goal document. No implementation yet. Phase 1 is read-only monitoring.

---

## 1. Why this exists

Bringing up Spark↔G1 teleop means juggling ~6 interdependent, polyglot, individually-flaky
processes across two machines. In practice we lose hours to:

- Processes **dying silently** (the C++ deploy vanished after ~a day) or **wedging while
  still "up"** (pico_manager stuck on a frozen body feed; the Orin zenoh bridge going stale
  after a robot reboot because it started before the robot's DDS was publishing).
- **No unified health view** — diagnosing meant manually running ZMQ topic probes, DDS
  rate probes, `docker ps`, `ss | grep 7447`, `ping`, and SSH-ing to the Orin.
- **Restart ordering / dependencies** — `roboticsservice` before `pico_manager`; Orin
  bridge needs the robot's DDS up; deploy needs `lowstate`; a stale Spark IP (RobotNet DHCP)
  silently breaks the PICO link.
- **Footguns** — `--output-type log` is a *dry run* that silently never actuates;
  `pkill -f` self-matches and kills the wrong shell; `A+B+X+Y` is the e-stop and also makes
  pico_manager `exit()`.

OTP supervision trees + LiveView are a near-ideal fit: declarative restart strategies with
backoff, dependency-aware restart, and a live dashboard with almost no UI plumbing.

---

## 2. Goals

1. **Single pane of glass**: live status of every rig process + the end-to-end data pipeline,
   updating ~1 Hz, reachable from a browser on the rig network.
2. **Correct liveness, not just "process exists"**: detect *wedged* states using the signals
   we learned (below), not just PID presence.
3. **Safe, dependency-aware lifecycle control**: start/stop/restart individual processes or
   the whole stack, in the right order, with backoff and restart-storm protection.
4. **Guaranteed child cleanup**: no orphaned/zombie OS processes when a supervisor restarts
   a child (the failure mode behind much of our manual `kill` pain).
5. **Safety-first actuation**: the deploy actuates a real robot — its start is gated and
   there is a prominent, fast E-STOP.
6. **Polished UX**: a genuinely nice dashboard (pipeline graph, per-process cards, telemetry
   sparklines), not a debug dump.

## 3. Non-goals

- Not rewriting the robot/teleop code (pico_manager, deploy, bridges) — RigPilot **wraps and
  orchestrates** them.
- Not replacing the C++ realtime control loop or the DDS/Zenoh transport.
- Not a general cluster manager — it's purpose-built for this rig.
- Phase 1 does **not** manage processes (monitor only); management lands in Phase 2.

---

## 4. The system being supervised

### 4.1 Process inventory

| Process | Host | Kind | Start command (current) |
|---|---|---|---|
| `roboticsservice` (`RoboticsServiceProcess`) | Spark | native binary | `/opt/apps/roboticsservice/runService.sh` (also auto-started by pico_manager) |
| `pico_manager_thread_server.py --manager` | Spark | python (`.venv_teleop`) | `…/.venv_teleop/bin/python gear_sonic/scripts/pico_manager_thread_server.py --manager` |
| `zenoh-spark-bridge` | Spark | docker | `eclipse/zenoh-bridge-dds` (see `gear_sonic_deploy/zenoh/`) |
| `g1_deploy_onnx_ref` | Spark | C++ via `deploy.sh` | `DEPLOY_YES=1 ./deploy.sh --input-type zmq_manager --output-type all g1zenoh` |
| `run_sim_loop.py` (sim only) | Spark | python (`.venv_sim`) | `…/.venv_sim/bin/python gear_sonic/scripts/run_sim_loop.py` |
| `zenoh-dds-bridge` | **Orin** `192.168.123.164` | docker (over SSH) | `ssh unitree@192.168.123.164 '~/zenoh-dds/start_bridge.sh'` |
| robot low-level service | robot `…123.161` | (the robot) | publishes `rt/lowstate`, reads `rt/lowcmd` |

### 4.2 Data flow & endpoints

```
 PICO headset ──wifi──► roboticsservice(:63901) ──(:60061)──► pico_manager
   (body+ctrl)                                          │ ZMQ PUB :5556 (pose / manager_state / planner)
                                                        ▼
                                                  g1_deploy_onnx_ref ── DDS rt/lowcmd ─┐
                                              (GR00T inference, lo, domain 0)          │
                                                        ▲ rt/lowstate                  │
                                       ZMQ :5557 g1_debug (telemetry)                  │
                                                        │                              ▼
   robot .161 ──DDS(eth0)──► Orin zenoh-dds-bridge ──Zenoh/TCP :7447──► zenoh-spark-bridge ──DDS(lo)──► deploy
```

Network: Spark `enP7s7 = 192.168.123.222/24` (wired to robot, never-default); Spark
`wlP9s9 = RobotNet` wifi (UniFi Express 7, **DHCP — drifts**, ideally reserve the lease);
Orin `eth0 = .164`, control board `.161`; DDS domain 0; Spark-local DDS on **`lo`**
(requires `lo` multicast + `239.255.0.0/16 dev lo` route).

---

## 5. Health & "wedged" detection (hard-won signals)

RigPilot's health checks must encode these, because PID-presence lies:

| Subject | Healthy signal | Wedged/broken signal |
|---|---|---|
| **PICO body feed** | `[PICO] LIVE pose_chg=True joint_ts_adv=True` (pico_manager log) | `STALE(poses frozen)`, or log stale >20 s |
| ⚠️ body timestamp | — | `body_ts_adv=False` is a **PICO-app bug — IGNORE it**, not a real failure |
| **pico_manager** | publishing on ZMQ `:5556`; log advancing | log idle >20 s; only `manager_state` (OFF/PLANNER), no `pose` in POSE mode |
| **deploy actuating** | `rt/lowcmd` ~500 Hz | `rt/lowcmd` = 0 → not engaged **or** `--output-type log` (dry-run!) |
| **robot → Spark** | `rt/lowstate` ~500–1000 Hz on `lo` | 0 → Orin bridge stale / robot not publishing |
| **Orin bridge** | `Route DDS->Zenoh (rt/lowstate)` present | no route (started before robot DDS — restart it) |
| **zenoh link** | 1 ESTAB conn Spark→`192.168.123.164:7447` | 0 |
| **robot** | ping `.161`/`.164`; battery OK | no ping (off/rebooting) |
| **PICO link** | roboticsservice peer connected; `find>0 miss=0` | `missing` climbing = network drop; app "WORKING" ≠ body live |

Notes baked in from this session:
- `A+B+X+Y` = e-stop **and** makes pico_manager `exit()` by design (expected, not a crash).
- The robot can be reachable (`.161` pings) yet not publishing low-level DDS right after boot.

---

## 6. Architecture

```
RigPilot.Application
├── RigPilot.ProcSupervisor            (DynamicSupervisor; restart strategy + backoff)
│   ├── ManagedProc :roboticsservice   ─┐
│   ├── ManagedProc :pico_manager       │  each wraps an OS process via **muontrap**
│   ├── ManagedProc :spark_bridge       │  (cgroup-scoped → guaranteed child cleanup)
│   ├── ManagedProc :deploy (gated)    ─┘
│   └── OrinBridge                      (remote: manages docker on .164 over SSH)
├── RigPilot.Telemetry
│   ├── SidecarPort                     (long-lived Python telemetry sidecar; JSON lines)
│   ├── HealthAggregator                (merges sidecar + proc states → rig snapshot)
│   └── Phoenix.PubSub broadcast        ("rig:status" → LiveViews)
└── RigPilotWeb.Endpoint (Phoenix)
    └── DashboardLive (LiveView)
```

### 6.1 ManagedProc (the OS-process wrapper)
A `GenServer` per process holding: spec (cmd, args, env, cwd, host), state
(`:down|:starting|:running|:wedged|:stopped`), restart count, ring buffer of recent log lines.
- Spawns via **`muontrap`** so a supervisor restart can't leave orphans (this is the
  make-or-break library — directly fixes the zombie/`pkill` mess).
- Periodic `health_check/1` callback uses the §5 signals (not just port liveness) to flag
  `:wedged` and trigger a restart.
- Streams stdout/stderr into the log ring buffer (also tee to `/tmp/<name>.log`).

### 6.2 Remote Orin bridge
`OrinBridge` manages the remote docker container over SSH:
`ssh unitree@192.168.123.164 '~/zenoh-dds/start_bridge.sh'`; health = "does a fresh
`DDS->Zenoh rt/lowstate` route exist AND is `lowstate` arriving on the Spark." Auto-restart
on robot reboot (detected via the lowstate gap).

### 6.3 Python telemetry sidecar (JSON contract)
Keep the cyclonedds/`xrobotoolkit_sdk` bindings in **Python** (where they live); expose a
small long-lived daemon that emits one JSON object per ~500 ms on stdout, consumed by an
Elixir `Port`:

```json
{
  "ts": 1716300000.12,
  "pico":   {"connected": true, "pose_chg": true, "joint_ts_adv": true,
             "body_ts_adv": false, "fps": 71.2, "miss": 0, "topics": ["pose","manager_state"]},
  "dds":    {"lowstate_hz": 503.1, "lowcmd_hz": 498.7, "secondary_imu_hz": 250.0},
  "zenoh":  {"spark_to_orin_sessions": 1},
  "net":    {"robot_161_ping_ms": 0.25, "robot_164_ping_ms": 0.14},
  "deploy": {"output_type": "all", "actuating": true}
}
```
Rationale: Elixir owns orchestration + UI; Python owns the SDK/DDS reads. Clean boundary,
no FFI into cyclonedds from the BEAM.

### 6.4 Dashboard (Phoenix LiveView)
- **Pipeline graph** across the top: `PICO → service → manager → ZMQ → deploy → zenoh → Orin → robot`,
  each hop green/amber/red from the health snapshot.
- **Per-process cards**: name, host, state badge, uptime, restart count, last N log lines,
  `Start · Stop · Restart` buttons (deploy's Start is gated — see §7).
- **Telemetry panel**: sparklines for `lowstate_hz`, `lowcmd_hz`, PICO `fps`; PICO body
  badge (LIVE/STALE) keyed on `pose_chg` (never on `body_ts`); zenoh sessions; robot ping;
  battery (if exposed).
- **Global E-STOP** button (always visible) + a banner when the deploy is actuating.

---

## 7. Safety

- `deploy` actuates a real robot. Its **Start is gated** behind an explicit confirm
  ("robot secured on gantry? clear space? hand on e-stop?") and shows the resolved
  `--output-type` (warn loudly if it's `log` = dry-run, since that silently no-ops).
- **E-STOP** kills the deploy (stops `rt/lowcmd` → robot dampers) within one tick; optionally
  also signals OFF. Must be the fastest path in the app.
- RigPilot must **never auto-start `deploy`** on boot/recovery without a human; auto-restart
  applies to non-actuating infra (bridges, services, pico_manager) only, unless explicitly
  armed.

---

## 8. Restart policy & ordering

- `rest_for_one` ordering so a dependency failure restarts everything downstream:
  `roboticsservice → pico_manager`, and `OrinBridge → spark_bridge → (deploy)` on the DDS path.
- Exponential backoff + max-restarts window (restart-storm protection); when the PICO is
  physically offline, back off rather than thrash (lesson from the bash watchdog).
- Pre-flight gates encoded as start conditions, e.g. deploy won't start until `lowstate_hz > 0`.

---

## 9. Tech stack

- **Elixir** + **OTP** (supervisors, GenServers, `DynamicSupervisor`).
- **Phoenix** + **LiveView** (real-time UI, no JS build needed).
- **`muontrap`** for OS-process spawning with guaranteed cleanup (or `erlexec` as fallback).
- **Phoenix.PubSub** for status fan-out.
- **Python telemetry sidecar** reusing `.venv_teleop` / `.venv_sim` (`xrobotoolkit_sdk`,
  `unitree_sdk2py` + the cyclonedds we built at `~/cyclonedds/install`).
- SSH (via `System.cmd` or `:erlexec`) for the Orin.
- (Chosen Elixir over Gleam for ecosystem maturity on exactly the hard parts — OS-process
  cleanup and live dashboards. Gleam is viable but younger here.)

---

## 10. Implementation phases

- **Phase 0 — skeleton**: `mix` umbrella (`rig_pilot`, `rig_pilot_web`), supervision tree
  stub, the Python sidecar + JSON contract, a LiveView showing raw sidecar JSON.
- **Phase 1 — read-only monitoring (ship first)**: full health aggregation + the polished
  dashboard (pipeline graph, cards, telemetry). **No process management** — zero risk, and
  it immediately replaces all our manual probing.
- **Phase 2 — managed lifecycle**: `ManagedProc` via muontrap for the Spark procs +
  `OrinBridge` over SSH; start/stop/restart from the UI; dependency-ordered restarts; backoff.
- **Phase 3 — safety + polish**: gated deploy start, global E-STOP, restart-storm protection,
  battery surfacing, persisted event log.

---

## 11. Risks & open questions

- **muontrap availability on aarch64 / the Spark** — verify the helper builds/runs; else
  `erlexec`. (Critical path: bad cleanup reproduces our zombie pain.)
- **roboticsservice ownership** — pico_manager currently auto-starts it via `runService.sh`;
  decide whether RigPilot owns it directly or treats it as pico_manager's child.
- **Sidecar vs deploy contending for DDS/SDK** — only one `xrobotoolkit_sdk` client may
  attach; the sidecar must read DDS (fine, multiple readers) but must **not** open a second
  PICO SDK session while pico_manager holds it (read PICO health from the log/ZMQ instead).
- **Where it runs / auth** — on the Spark; bind the dashboard to the rig LAN; basic auth.
- **Battery telemetry** — is SOC exposed in `rt/lowstate`? If so, surface it (this whole
  detour started with a low battery).
- **Spark IP drift** — RigPilot should surface the current `wlP9s9` IP and warn the PICO
  app target may be stale; a UniFi DHCP reservation is the real fix.

---

## 12. Dependency & state model

"Up/down" is not enough — the two failures that actually cost us time were a *tracker
battery* red-herring and a *robot SOC* power-cycle. So we model both a dependency DAG and
rich per-entity state.

### 12.1 Dependencies = a DAG with gates (not the linear pipeline)

Each managed entity declares two things:
- **`deps`** — other *processes* that must be running first (start order + cascade).
- **`gates`** — *health predicates* that must hold before it may reach `:running`.

```
roboticsservice ──► pico_manager ───────────────┐
                                                  ├──► deploy ──► robot (actuates)
robot ──► orin_bridge ──► spark_bridge ──► lowstate ┘
```
Examples:
- `deploy.deps  = [pico_manager, spark_bridge]`
- `deploy.gates = [lowstate_hz > 0, pico.pose_available, output_type != :log]`
- `orin_bridge.gates = [robot.reachable, robot.publishing_dds]`
- **mutual exclusion**: `{sim}` ⊻ `{real-robot path}` (can't both drive DDS)

Responsibilities split:
- **OTP supervisor** → restart *mechanics* (backoff, restart-storm caps).
- **Dependency layer** (in HealthAggregator) → *policy*: topological start order,
  start-gating (won't start until deps+gates green), and **cascade** — when a node goes
  unhealthy, mark dependents `:blocked` with a `blocked_by:` reason instead of letting them
  flap. (`rest_for_one` only covers a linear chain; our DAG needs explicit edges.)

UI: DAG edges turn red on a broken dependency; a blocked card shows `⛔ blocked by orin_bridge`
rather than a misleading "down".

### 12.2 Rich per-entity state

Every entity (process *or* external device) carries:
```elixir
%Health{
  status:  :running,   # :down|:starting|:running|:wedged|:degraded|:blocked|:stopped
  level:   :warn,      # :ok | :warn | :crit  — rolled up from metrics
  reasons: ["L ankle tracker 14%"],
  metrics: %{tracker_l_batt: 14, body_feed: :live, fps: 71}
}
```
`level` comes from **threshold rules** over `metrics`, rolled up into the card badge. The
rules encode the exact failure modes we hit:

| Entity | Metrics surfaced | Rule examples |
|---|---|---|
| **PICO** | headset / controller / **ankle-tracker** battery %, full-body active, `pose_chg` | tracker <20% → warn, <10% → crit |
| **robot** | **SOC / voltage**, per-motor fault code, low-level/debug mode, joint temps | SOC <15% → crit; any motor `0x40000` → crit "re-enable low-level" |
| **network** | Spark `wlan0` IP, zenoh sessions, lowstate **jitter** | IP changed → warn "PICO target stale"; jitter >50 ms → warn |
| **deploy** | `output_type`, `lowcmd_hz` | `output_type == log` → warn "dry-run, won't actuate" |

So "is the PICO out of battery" = `pico.metrics.tracker_l_batt` → rule → `level: :crit` → a
red battery chip on the PICO node with reason "L tracker 8%".

### 12.3 Data sources (Phase-2 sidecar work)

- **Robot SOC + motor faults** → already in `rt/lowstate` (HG `LowState_` power/BMS + per-motor
  status — we literally saw the `0x40000` faults). Low lift, high value.
- **PICO headset/controller/tracker batteries** → in the XRoboToolkit **device-state JSON**,
  but the current pybind only parses poses/buttons. Extend the binding/sidecar to pull battery
  *if the app includes those fields* (same JSON where the `body_ts` bug lives — must confirm).
- All of it rides the existing **`"rig:status"` PubSub contract** as extra fields, so the
  LiveView shape doesn't change.

## 13. Safe restart & actuation interlocks

The hazard is **discontinuity at re-engage**, not the restart itself: a controller that
resumes commanding a target far from the robot's *current* pose makes the PD loop slam to it
("the robot instantly snaps to your pose; a large mismatch causes aggressive motion"). Every
rule below ensures actuation never resumes from a jump.

**Responsibility split**
- **RigPilot** (orchestrator — never in the realtime path): restart *ordering*, health
  *gates*, *graceful* stop signaling, and refusing to auto-engage actuation.
- **deploy** (must support; RigPilot verifies): init-from-measured-pose + soft-start ramp on
  engage; drop to damping when its inputs gap.
- **robot firmware** (backstop): `rt/lowcmd` timeout → damping.

**Tiers**
- *Non-actuating* (roboticsservice, pico_manager, zenoh bridges, telemetry): emit no
  `rt/lowcmd`, so safe to restart — but the deploy must detect the resulting `lowstate`/pose
  gap and disengage to damping, never command on stale state.
- *Actuating* (deploy — the only `rt/lowcmd` source): restart **only while disengaged**; it
  must come back **disengaged**; the first engaged command must be continuous
  (`CALIB` / init-from-measured).

**Ordered restart sequence** (any restart touching the control path)
1. **Disengage first** — signal the deploy to stop/OFF (it ramps to damping); confirm
   `lowcmd` ceased. *Not* a SIGKILL.
2. **Stop** downstream→upstream (deploy → spark_bridge → orin_bridge …).
3. **Start** upstream→downstream, **gated** — each stage waits for its dependency to be
   healthy (e.g. deploy won't start until `lowstate_hz > 0`).
4. **Come up disengaged** — never auto-engage.
5. **Human re-arm** — operator re-engages; engage runs `CALIB` (init from measured) → zero
   initial error → no jump.

Between steps 1 and 5 the robot is in damping, so steps 2–4 cannot produce a sudden command.

**Stop strategy** (`ManagedProc.stop_strategy`)
- `deploy` → `:graceful` — send disengage, wait a timeout for `lowcmd` to cease, *then*
  SIGTERM via muontrap. Never SIGKILL an actively-commanding deploy.
- bridges / services → `:term`.

**Interlocks**
- Never auto-engage actuation — orchestrated restarts leave the deploy **disarmed**;
  re-engage is human-only.
- Deploy Start stays behind the arm/confirm gate; post-restart it shows "ready — disengaged".
- Optional **pre-engage alignment check**: warn/block if the streamed target ≫ measured pose.
- **Rehearse** a restart with `--output-type log` (dry-run, zero actuation) before going live.
- **E-STOP** is an independent, always-available fast path (cut `rt/lowcmd` → damping) that
  bypasses the orchestration.

**Honest precondition:** safe only if the deploy supports (a) graceful stop-to-damping and
(b) init-from-measured + soft-start. If not, RigPilot treats a deploy stop as
"firmware-timeout damping" and **must not** offer a hot restart — it surfaces the limitation
rather than pretend.

**UI:** two actions — *Restart infra* (one-click; warns it forces deploy disengage if in-path)
vs *Restart control chain* (the gated sequence shown as a live step-checklist); plus an
**engaged / disengaged / armed** indicator on the deploy node throughout.

## 14. Definition of done (Phase 1)

Open a browser on the rig, and within 2 s see — without running a single shell command —
whether each process is up/wedged/down, whether the PICO body feed is truly LIVE, the
`lowstate`/`lowcmd` rates, the zenoh link, and a green/red end-to-end pipeline. That alone
would have turned this session's multi-hour debugging into a glance.
