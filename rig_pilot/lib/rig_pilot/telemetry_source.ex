defmodule RigPilot.TelemetrySource do
  @moduledoc """
  PLACEHOLDER — wire to the Python sidecar `Port` in Phase 2.

  A `GenServer` that, on a ~1s timer, generates **plausible FAKE telemetry**
  matching the JSON contract in §6.3 of the design doc and broadcasts it on the
  `"rig:status"` PubSub topic as `{:telemetry, snapshot}`.

  Nothing here reads DDS, ZMQ, the PICO SDK, or the network. The numbers are
  random walks so the dashboard sparklines move.

  Contract produced (mirrors the sidecar's stdout JSON):

      %{
        ts: float,
        pico:   %{connected, pose_chg, joint_ts_adv, body_ts_adv, fps, miss, topics,
                  headset_batt, controller_l_batt, controller_r_batt,
                  tracker_l_batt, tracker_r_batt},
        robot:  %{soc, voltage, motor_fault},
        dds:    %{lowstate_hz, lowcmd_hz, secondary_imu_hz},
        zenoh:  %{spark_to_orin_sessions},
        net:    %{robot_161_ping_ms, robot_164_ping_ms},
        deploy: %{output_type, actuating}
      }

  Battery / SOC / motor-fault fields are FAKE placeholders here. In Phase 2 they
  arrive from the real sidecar: robot SOC/voltage/per-motor faults come straight
  out of `rt/lowstate` (HG `LowState_` BMS + motor status — that's where we saw
  the `0x40000` faults); the PICO headset/controller/tracker batteries come from
  the XRoboToolkit device-state JSON once the pybind is extended to parse them.

  ## Phase 2 wiring point
  Replace `gen/1` (and the seeded state) with a `Port` opened on the long-lived
  Python telemetry daemon, decode one JSON line per message, and merge with
  `RigPilot.ManagedProc.all/0` in a HealthAggregator. The broadcast shape stays
  the same so the LiveView is unaffected.
  """

  use GenServer

  @pubsub RigPilot.PubSub
  @topic "rig:status"
  @interval_ms 1_000

  # ── public API ──────────────────────────────────────────────────────────

  def start_link(_opts) do
    GenServer.start_link(__MODULE__, :ok, name: __MODULE__)
  end

  @doc "Latest fake telemetry snapshot."
  def latest, do: GenServer.call(__MODULE__, :latest)

  # ── GenServer ───────────────────────────────────────────────────────────

  @impl true
  def init(:ok) do
    state = gen(seed())
    schedule()
    {:ok, state}
  end

  @impl true
  def handle_call(:latest, _from, state), do: {:reply, state, state}

  @impl true
  def handle_info(:tick, state) do
    new_state = gen(state)
    Phoenix.PubSub.broadcast(@pubsub, @topic, {:telemetry, new_state})
    schedule()
    {:noreply, new_state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # ── fake data generation (NO real probing) ────────────────────────────────

  defp seed do
    %{
      lowstate_hz: 503.0,
      lowcmd_hz: 498.0,
      secondary_imu_hz: 250.0,
      fps: 71.0,
      miss: 0,
      ping161: 0.25,
      ping164: 0.14,
      # battery / power state (0–100). tracker_l drains to demo :crit.
      headset_batt: 84.0,
      controller_l_batt: 67.0,
      controller_r_batt: 72.0,
      tracker_l_batt: 35.0,
      tracker_r_batt: 58.0,
      soc: 78.0,
      voltage: 57.6,
      motor_fault: :none
    }
  end

  defp gen(prev) do
    lowstate = walk(prev[:lowstate_hz] || 503.0, 490.0, 1000.0, 12.0)
    lowcmd = walk(prev[:lowcmd_hz] || 498.0, 480.0, 510.0, 10.0)
    imu = walk(prev[:secondary_imu_hz] || 250.0, 240.0, 260.0, 4.0)
    fps = walk(prev[:fps] || 71.0, 55.0, 90.0, 3.0)
    ping161 = walk(prev[:ping161] || 0.25, 0.1, 1.2, 0.15) |> max(0.0)
    ping164 = walk(prev[:ping164] || 0.14, 0.1, 1.2, 0.12) |> max(0.0)

    # Occasionally bump a "miss" to demonstrate the amber state, then settle.
    miss =
      cond do
        :rand.uniform(40) == 1 -> (prev[:miss] || 0) + 1
        :rand.uniform(4) == 1 -> 0
        true -> prev[:miss] || 0
      end

    # PICO body feed: LIVE most of the time (keyed on pose_chg, NOT body_ts).
    pose_chg = :rand.uniform(12) != 1

    # ── batteries (fake): most are stable random walks; the LEFT ankle tracker
    # slowly drains and periodically dips below 10% to demonstrate the :crit
    # battery rule (the tracker-battery red-herring that cost us a session).
    headset_batt = drain_walk(prev[:headset_batt] || 84.0, 60.0, 100.0, 0.4)
    controller_l_batt = drain_walk(prev[:controller_l_batt] || 67.0, 30.0, 100.0, 0.6)
    controller_r_batt = drain_walk(prev[:controller_r_batt] || 72.0, 30.0, 100.0, 0.6)
    tracker_r_batt = drain_walk(prev[:tracker_r_batt] || 58.0, 20.0, 100.0, 0.5)

    # tracker_l: drain steadily; when it bottoms out, jump back up (battery swap).
    tracker_l_prev = prev[:tracker_l_batt] || 35.0

    tracker_l_batt =
      if tracker_l_prev <= 6.0 do
        # "swapped" / recharged
        90.0 + :rand.uniform() * 8.0
      else
        max(tracker_l_prev - (0.4 + :rand.uniform() * 1.2), 3.0)
      end

    # ── robot power (fake): SOC mostly healthy; occasionally crashes < 15% and
    # raises a motor fault to demonstrate :crit, then recovers (power-cycle).
    soc_prev = prev[:soc] || 78.0
    prev_fault = prev[:motor_fault] || :none

    {soc, motor_fault} =
      cond do
        # already in a fault dip → climb back out and clear the fault
        soc_prev < 15.0 ->
          {min(soc_prev + 4.0 + :rand.uniform() * 3.0, 80.0), :none}

        # rare event: drop into a crit SOC dip + a motor fault code
        :rand.uniform(60) == 1 ->
          {8.0 + :rand.uniform() * 5.0, "0x40000"}

        # normal slow drain/charge wobble; keep any lingering fake fault
        true ->
          {drain_walk(soc_prev, 60.0, 95.0, 0.7), prev_fault}
      end

    voltage = walk(prev[:voltage] || 57.6, 48.0, 58.8, 0.2)

    %{
      ts: System.system_time(:millisecond) / 1000.0,
      pico: %{
        connected: true,
        pose_chg: pose_chg,
        joint_ts_adv: pose_chg,
        # body_ts_adv is a known PICO-app bug — always false, must be IGNORED.
        body_ts_adv: false,
        fps: round1(fps),
        miss: miss,
        topics: ["pose", "manager_state"],
        # battery % (0–100), placeholder
        headset_batt: round(headset_batt),
        controller_l_batt: round(controller_l_batt),
        controller_r_batt: round(controller_r_batt),
        tracker_l_batt: round(tracker_l_batt),
        tracker_r_batt: round(tracker_r_batt)
      },
      robot: %{
        soc: round(soc),
        voltage: round1(voltage),
        motor_fault: motor_fault
      },
      dds: %{
        lowstate_hz: round1(lowstate),
        lowcmd_hz: round1(lowcmd),
        secondary_imu_hz: round1(imu)
      },
      zenoh: %{spark_to_orin_sessions: 1},
      net: %{
        robot_161_ping_ms: round2(ping161),
        robot_164_ping_ms: round2(ping164)
      },
      deploy: %{output_type: "all", actuating: true},
      # keep raw scalars around for the next random walk
      lowstate_hz: lowstate,
      lowcmd_hz: lowcmd,
      secondary_imu_hz: imu,
      fps: fps,
      miss: miss,
      ping161: ping161,
      ping164: ping164,
      headset_batt: headset_batt,
      controller_l_batt: controller_l_batt,
      controller_r_batt: controller_r_batt,
      tracker_l_batt: tracker_l_batt,
      tracker_r_batt: tracker_r_batt,
      soc: soc,
      voltage: voltage,
      motor_fault: motor_fault
    }
  end

  # Slow one-directional-ish drain: biased downward but can wobble up a little,
  # clamped to [lo, hi]. Used for batteries / SOC that should trend down.
  defp drain_walk(v, lo, hi, step) do
    delta = (:rand.uniform() - 0.65) * 2 * step
    (v + delta) |> min(hi) |> max(lo)
  end

  # Random walk clamped to [lo, hi].
  defp walk(v, lo, hi, step) do
    (v + (:rand.uniform() - 0.5) * 2 * step) |> min(hi) |> max(lo)
  end

  defp round1(v), do: Float.round(v, 1)
  defp round2(v), do: Float.round(v, 2)

  defp schedule, do: Process.send_after(self(), :tick, @interval_ms)
end
