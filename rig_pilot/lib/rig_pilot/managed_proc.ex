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
            gated: false

  @type status :: :down | :starting | :running | :wedged | :stopped

  # ── static definitions for every managed process in the doc ──
  @specs %{
    roboticsservice: %{
      label: "roboticsservice",
      host: "Spark",
      kind: :native,
      gated: false,
      seed_status: :running,
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
      logs: ["[sim] stopped"]
    },
    orin_bridge: %{
      label: "orin_bridge (zenoh-dds)",
      host: "Orin 192.168.123.164",
      kind: :remote,
      gated: false,
      seed_status: :running,
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
      gated: state.gated
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
