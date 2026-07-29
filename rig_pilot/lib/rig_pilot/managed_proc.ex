defmodule RigPilot.ManagedProc do
  @moduledoc """
  PLACEHOLDER — wire to muontrap/SSH in Phase 2.

  A `GenServer` that *represents* one managed OS subprocess of the rig
  (`roboticsservice`, `pico_manager`, `zenoh_spark_bridge`, `deploy`, `sim`,
  `orin_bridge`). For Phase 1 this holds **in-memory state only** and does NOT
  spawn, kill, ssh, or probe anything real.

  State held:

    * `name`          — atom identifier
    * `label`         — human display name
    * `host`          — where the real process would run (Spark / Orin / robot)
    * `kind`          — :native | :python | :docker | :cpp | :remote
    * `status`        — :down | :starting | :running | :wedged | :stopped
    * `uptime`        — seconds in :running (computed from started_at)
    * `restart_count` — number of times start/restart was invoked
    * `recent_logs`   — ring buffer of fake log lines (most recent last)
    * `output_type`   — deploy-only; "all" | "log" | "zenoh" (dry-run if "log")
    * `gated`         — true for processes whose Start requires confirmation

  `start/1`, `stop/1`, `restart/1` simply mutate this state (flip to `:starting`
  then `:running` after a short timer), append a fake log line, and broadcast on
  the `"rig:status"` PubSub topic. They MUST NOT touch the live rig.

  ## Phase 2 wiring point
  Replace the body of `do_start/1`, `do_stop/1` and the `:health_check` handler
  with real `MuonTrap.Daemon` (Spark procs) / `System.cmd("ssh", ...)` (Orin)
  calls, and stream real stdout/stderr into `recent_logs`. The public API and
  PubSub contract below stay identical, so the LiveView needs no changes.
  """

  use GenServer
  require Logger

  @pubsub RigPilot.PubSub
  @topic "rig:status"
  @max_logs 8

  # ── how long the fake "starting" phase lasts before flipping to running ──
  @start_delay_ms 1_200
  # ── how often we recompute uptime so the card ticks ──
  @tick_ms 1_000

  defstruct name: nil,
            label: nil,
            host: nil,
            kind: :native,
            status: :down,
            started_at: nil,
            uptime: 0,
            restart_count: 0,
            recent_logs: [],
            output_type: nil,
            gated: false,
            deps: [],
            gates: []

  @type status :: :down | :starting | :running | :wedged | :stopped

  # ── static definitions for every managed process in the doc ──
  #
  # `deps`  — other *processes* that must be :running first (§12.1 start order
  #           + cascade). If a dep is not running, this entity is :blocked.
  # `gates` — health predicates (atoms, resolved in `gate_ok?/3`) that must hold
  #           before the entity may stay :running. A failed gate also blocks.
  #
  # The dependency DAG from §12.1:
  #   roboticsservice ─► pico_manager ─────────────────┐
  #                                                      ├─► deploy ─► robot
  #   robot ─► orin_bridge ─► spark_bridge ─► lowstate ─┘
  @specs %{
    roboticsservice: %{
      label: "roboticsservice",
      host: "Spark",
      kind: :native,
      gated: false,
      seed_status: :running,
      deps: [],
      gates: [],
      logs: [
        "[svc] runService.sh up, listening :63901",
        "[svc] PICO peer connected, find=1 miss=0"
      ]
    },
    pico_manager: %{
      label: "pico_manager",
      host: "Spark",
      kind: :python,
      gated: false,
      seed_status: :running,
      deps: [:roboticsservice],
      gates: [:pico_connected],
      logs: [
        "[PICO] LIVE pose_chg=True joint_ts_adv=True",
        "[zmq] PUB :5556 pose advancing"
      ]
    },
    zenoh_spark_bridge: %{
      label: "zenoh_spark_bridge",
      host: "Spark (docker)",
      kind: :docker,
      gated: false,
      seed_status: :running,
      deps: [:orin_bridge],
      gates: [],
      logs: [
        "[zenoh] bridge up, DDS on lo domain 0",
        "[zenoh] Route Zenoh->DDS (rt/lowcmd)"
      ]
    },
    deploy: %{
      label: "deploy (g1_deploy_onnx_ref)",
      host: "Spark",
      kind: :cpp,
      gated: true,
      seed_status: :stopped,
      output_type: "all",
      deps: [:pico_manager, :zenoh_spark_bridge],
      gates: [:lowstate_present, :pico_pose_available, :output_actuates],
      logs: [
        "[deploy] idle — not engaged",
        "[deploy] waiting for lowstate_hz > 0"
      ]
    },
    sim: %{
      label: "sim (run_sim_loop)",
      host: "Spark",
      kind: :python,
      gated: false,
      seed_status: :stopped,
      deps: [],
      gates: [],
      logs: ["[sim] stopped"]
    },
    orin_bridge: %{
      label: "orin_bridge (zenoh-dds)",
      host: "Orin 192.168.123.164",
      kind: :remote,
      gated: false,
      seed_status: :running,
      deps: [],
      gates: [:robot_reachable, :robot_publishing],
      logs: [
        "[orin] ssh ok, container up",
        "[orin] Route DDS->Zenoh (rt/lowstate)"
      ]
    }
  }

  # ── public API ──────────────────────────────────────────────────────────

  def specs, do: @specs
  def names, do: Map.keys(@specs)

  def start_link(name) when is_atom(name) do
    GenServer.start_link(__MODULE__, name, name: via(name))
  end

  @doc "Current immutable snapshot of this process's state."
  def get(name), do: GenServer.call(via(name), :get)

  @doc "Snapshot of every managed process, keyed by name."
  def all do
    for name <- names(), into: %{}, do: {name, get(name)}
  end

  @doc """
  Like `all/0`, but each snapshot is decorated with the §12 dependency & state
  model given a telemetry map:

    * `:level`      — `:ok | :warn | :crit`, rolled up from this entity's metrics
    * `:reasons`    — human strings explaining the level / block
    * `:status`     — possibly overridden to `:blocked` when a dep is down or a
                      gate fails (the §12.1 cascade)
    * `:blocked_by` — the dep name (or gate label) that caused the block, else nil

  Pure: this is the placeholder for the Phase-2 HealthAggregator *policy* layer.
  OTP supervisors own restart *mechanics*; this owns dependency cascade + gating.
  """
  def all(tele) do
    raw = all()
    for {name, snap} <- raw, into: %{}, do: {name, decorate(snap, raw, tele)}
  end

  @doc "PLACEHOLDER start — mutates state to :starting then :running. No spawn."
  def start(name), do: GenServer.call(via(name), :start)

  @doc "PLACEHOLDER stop — mutates state to :stopped. No kill."
  def stop(name), do: GenServer.call(via(name), :stop)

  @doc "PLACEHOLDER restart — stop then start, bumps restart_count. No spawn."
  def restart(name), do: GenServer.call(via(name), :restart)

  defp via(name), do: {:via, Registry, {RigPilot.ProcRegistry, name}}

  # ── GenServer ───────────────────────────────────────────────────────────

  @impl true
  def init(name) do
    spec = Map.fetch!(@specs, name)
    seed = Map.get(spec, :seed_status, :down)

    state =
      %__MODULE__{
        name: name,
        label: spec.label,
        host: spec.host,
        kind: spec.kind,
        status: seed,
        gated: Map.get(spec, :gated, false),
        output_type: Map.get(spec, :output_type),
        deps: Map.get(spec, :deps, []),
        gates: Map.get(spec, :gates, []),
        recent_logs: Map.get(spec, :logs, []),
        started_at: if(seed == :running, do: now() - :rand.uniform(5_400), else: nil),
        restart_count: 0
      }

    schedule_tick()
    {:ok, state}
  end

  @impl true
  def handle_call(:get, _from, state) do
    {:reply, snapshot(state), state}
  end

  def handle_call(:start, _from, state) do
    {:reply, :ok, do_start(state)}
  end

  def handle_call(:stop, _from, state) do
    {:reply, :ok, do_stop(state)}
  end

  def handle_call(:restart, _from, state) do
    # PLACEHOLDER restart = stop, then start after a beat.
    state =
      state
      |> do_stop()
      |> append_log("[#{state.name}] restart requested")
      |> Map.update!(:restart_count, &(&1 + 1))

    Process.send_after(self(), :do_start_after_restart, 300)
    {:reply, :ok, state}
  end

  @impl true
  def handle_info(:do_start_after_restart, state) do
    {:noreply, do_start(state)}
  end

  # Fake "starting" -> "running" transition.
  def handle_info(:finish_start, %{status: :starting} = state) do
    state =
      state
      |> Map.put(:status, :running)
      |> Map.put(:started_at, now())
      |> append_log("[#{state.name}] up and #{running_verb(state)}")

    broadcast()
    {:noreply, state}
  end

  def handle_info(:finish_start, state), do: {:noreply, state}

  def handle_info(:tick, state) do
    schedule_tick()

    state = maybe_flap(state)

    new_uptime =
      if state.status == :running and state.started_at,
        do: now() - state.started_at,
        else: 0

    {:noreply, %{state | uptime: new_uptime}}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # ── placeholder lifecycle helpers (NO real spawning) ──────────────────────
  #
  # PHASE 2: replace the bodies below with MuonTrap.Daemon.start_link/SSH and
  # wire health_check to the §5 wedged-detection signals from the design doc.

  defp do_start(%{status: :running} = state), do: state

  defp do_start(state) do
    state =
      state
      |> Map.put(:status, :starting)
      |> Map.put(:started_at, nil)
      |> Map.update!(:restart_count, &(&1 + 1))
      |> append_log("[#{state.name}] starting (placeholder, no OS spawn)")

    Process.send_after(self(), :finish_start, @start_delay_ms)
    broadcast()
    state
  end

  defp do_stop(state) do
    state =
      state
      |> Map.put(:status, :stopped)
      |> Map.put(:started_at, nil)
      |> Map.put(:uptime, 0)
      |> append_log("[#{state.name}] stopped (placeholder, no kill)")

    broadcast()
    state
  end

  # ── demo flap (NO real probing) ───────────────────────────────────────────
  #
  # PHASE 2: delete this. Health/wedged comes from the §5 signals + the sidecar.
  # Here we deliberately let ONE dependency (`orin_bridge`) occasionally go
  # `:wedged` for a few seconds so downstream cards (zenoh_spark_bridge → deploy)
  # visibly render "⛔ blocked by orin_bridge", then it self-heals.
  defp maybe_flap(%{name: :orin_bridge, status: :running} = state) do
    if :rand.uniform(45) == 1 do
      state
      |> Map.put(:status, :wedged)
      |> append_log("[orin] route DDS->Zenoh (rt/lowstate) went STALE — wedged")
      |> tap_broadcast()
    else
      state
    end
  end

  defp maybe_flap(%{name: :orin_bridge, status: :wedged} = state) do
    if :rand.uniform(6) == 1 do
      state
      |> Map.put(:status, :running)
      |> Map.put(:started_at, now())
      |> append_log("[orin] route DDS->Zenoh (rt/lowstate) restored")
      |> tap_broadcast()
    else
      state
    end
  end

  defp maybe_flap(state), do: state

  defp tap_broadcast(state) do
    broadcast()
    state
  end

  # ── §12 dependency & state model (pure; placeholder HealthAggregator) ──────
  #
  # Computes per-entity {status, level, reasons, blocked_by}. `level` is a
  # threshold rollup over telemetry metrics; the cascade marks an entity
  # `:blocked` when a `dep` is not running or a `gate` predicate fails.

  defp decorate(snap, all_procs, tele) do
    {level, level_reasons} = level_for(snap, tele)

    {blocked_by, block_reason} = blocked_by(snap, all_procs, tele)

    {status, reasons} =
      cond do
        # a down/stopped/wedged proc is its own problem, not "blocked"
        snap.status in [:down, :stopped, :wedged, :starting] ->
          {snap.status, level_reasons}

        # running but a dependency/gate failed → cascade to :blocked
        blocked_by != nil ->
          {:blocked, [block_reason | level_reasons]}

        true ->
          {snap.status, level_reasons}
      end

    # a block forces at least :warn (use :crit if a dep is fully down)
    level =
      cond do
        status == :blocked and block_dep_down?(snap, all_procs) -> :crit
        status == :blocked -> max_level(level, :warn)
        true -> level
      end

    Map.merge(snap, %{
      status: status,
      level: level,
      reasons: reasons,
      blocked_by: blocked_by
    })
  end

  # ── level: threshold rollup over this entity's telemetry metrics ──────────

  defp level_for(%{name: :pico_manager}, tele) do
    rollup([
      batt_rule("headset", tele.pico.headset_batt),
      batt_rule("L ctrl", tele.pico.controller_l_batt),
      batt_rule("R ctrl", tele.pico.controller_r_batt),
      batt_rule("L ankle tracker", tele.pico.tracker_l_batt),
      batt_rule("R ankle tracker", tele.pico.tracker_r_batt),
      if(tele.pico.connected, do: nil, else: {:crit, "PICO not connected"})
    ])
  end

  defp level_for(%{name: name}, tele) when name in [:zenoh_spark_bridge, :orin_bridge] do
    # robot power surfaces on the DDS path entities
    rollup([
      soc_rule(tele.robot.soc),
      motor_fault_rule(tele.robot.motor_fault)
    ])
  end

  defp level_for(%{name: :deploy} = snap, tele) do
    rollup([
      soc_rule(tele.robot.soc),
      motor_fault_rule(tele.robot.motor_fault),
      if(snap.output_type == "log",
        do: {:warn, "output-type log — dry-run, won't actuate"},
        else: nil
      )
    ])
  end

  defp level_for(_snap, _tele), do: {:ok, []}

  # battery thresholds (§12.2): <10 crit, <15 warn-ish; here ≥40 ok / 15–39 warn / <15 crit
  defp batt_rule(_label, pct) when pct >= 40, do: nil
  defp batt_rule(label, pct) when pct >= 15, do: {:warn, "#{label} #{pct}%"}
  defp batt_rule(label, pct), do: {:crit, "#{label} #{pct}%"}

  defp soc_rule(soc) when soc >= 25, do: nil
  defp soc_rule(soc) when soc >= 15, do: {:warn, "robot SOC #{soc}%"}
  defp soc_rule(soc), do: {:crit, "robot SOC #{soc}% — power low"}

  defp motor_fault_rule(:none), do: nil
  defp motor_fault_rule(code), do: {:crit, "motor fault #{code} — re-enable low-level"}

  defp rollup(rules) do
    parsed = Enum.reject(rules, &is_nil/1)
    reasons = Enum.map(parsed, fn {_lvl, r} -> r end)
    level = Enum.reduce(parsed, :ok, fn {lvl, _}, acc -> max_level(acc, lvl) end)
    {level, reasons}
  end

  defp max_level(:crit, _), do: :crit
  defp max_level(_, :crit), do: :crit
  defp max_level(:warn, _), do: :warn
  defp max_level(_, :warn), do: :warn
  defp max_level(_, _), do: :ok

  # ── cascade: which dep/gate (if any) is blocking a running entity ──────────

  defp blocked_by(snap, all_procs, tele) do
    down_dep =
      Enum.find(snap.deps, fn dep ->
        case all_procs[dep] do
          %{status: :running} -> false
          _ -> true
        end
      end)

    failed_gate = Enum.find(snap.gates, fn g -> not gate_ok?(g, tele, all_procs) end)

    cond do
      down_dep != nil ->
        {down_dep, "blocked by #{down_dep} (#{dep_state(all_procs[down_dep])})"}

      failed_gate != nil ->
        {gate_label(failed_gate), "gate failed: #{gate_label(failed_gate)}"}

      true ->
        {nil, nil}
    end
  end

  # true only when the block is caused by a dep that is fully down (→ :crit)
  defp block_dep_down?(snap, all_procs) do
    Enum.any?(snap.deps, fn dep ->
      case all_procs[dep] do
        %{status: s} when s in [:down, :stopped] -> true
        _ -> false
      end
    end)
  end

  defp dep_state(nil), do: "missing"
  defp dep_state(%{status: s}), do: s

  # gate predicates resolved from telemetry / other procs (§12.1 examples)
  defp gate_ok?(:lowstate_present, tele, _procs), do: tele.dds.lowstate_hz > 0
  defp gate_ok?(:pico_pose_available, tele, _procs), do: tele.pico.pose_chg == true
  defp gate_ok?(:output_actuates, _tele, procs) do
    case procs[:deploy] do
      %{output_type: ot} -> ot != "log"
      _ -> true
    end
  end
  defp gate_ok?(:pico_connected, tele, _procs), do: tele.pico.connected == true
  defp gate_ok?(:robot_reachable, tele, _procs), do: tele.net.robot_161_ping_ms > 0
  defp gate_ok?(:robot_publishing, tele, _procs), do: tele.dds.lowstate_hz > 0
  defp gate_ok?(_unknown, _tele, _procs), do: true

  defp gate_label(:lowstate_present), do: "lowstate_hz > 0"
  defp gate_label(:pico_pose_available), do: "pico.pose_available"
  defp gate_label(:output_actuates), do: "output_type != log"
  defp gate_label(:pico_connected), do: "pico.connected"
  defp gate_label(:robot_reachable), do: "robot reachable"
  defp gate_label(:robot_publishing), do: "robot publishing DDS"
  defp gate_label(other), do: to_string(other)

  defp running_verb(%{kind: :docker}), do: "container healthy"
  defp running_verb(%{kind: :remote}), do: "remote container healthy"
  defp running_verb(%{kind: :python}), do: "loop running"
  defp running_verb(%{name: :deploy}), do: "actuating"
  defp running_verb(_), do: "running"

  defp append_log(state, line) do
    stamped = "#{ts()} #{line}"
    logs = (state.recent_logs ++ [stamped]) |> Enum.take(-@max_logs)
    %{state | recent_logs: logs}
  end

  defp snapshot(state) do
    %{
      name: state.name,
      label: state.label,
      host: state.host,
      kind: state.kind,
      status: state.status,
      uptime: state.uptime,
      restart_count: state.restart_count,
      recent_logs: state.recent_logs,
      output_type: state.output_type,
      gated: state.gated,
      deps: state.deps,
      gates: state.gates
    }
  end

  defp broadcast, do: Phoenix.PubSub.broadcast(@pubsub, @topic, :procs_changed)

  defp schedule_tick, do: Process.send_after(self(), :tick, @tick_ms)

  defp now, do: System.system_time(:second)

  defp ts do
    {_, {h, m, s}} = :calendar.local_time()
    :io_lib.format("~2..0B:~2..0B:~2..0B", [h, m, s]) |> List.to_string()
  end
end
