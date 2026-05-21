defmodule RigPilotWeb.DashboardLive do
  @moduledoc """
  RigPilot control-center dashboard (Phase 1, placeholder backends).

  Subscribes to the `"rig:status"` PubSub topic and renders:

    * an end-to-end pipeline graph (PICO → … → robot),
    * a per-process card grid with working Start/Stop/Restart buttons
      (deploy's Start is gated behind a confirm modal),
    * a telemetry panel with numeric tiles + sparklines,
    * an always-visible E-STOP.

  Every action calls the PLACEHOLDER `RigPilot.ManagedProc` / E-STOP flag — no
  real rig process is ever touched. Phase 2 swaps the backends, not this view.
  """
  use RigPilotWeb, :live_view

  alias RigPilot.{ManagedProc, TelemetrySource}

  @topic "rig:status"
  # how many points the sparklines keep
  @history 40

  @impl true
  def mount(_params, _session, socket) do
    if connected?(socket), do: Phoenix.PubSub.subscribe(RigPilot.PubSub, @topic)

    tele = TelemetrySource.latest()

    socket =
      socket
      |> assign(:page_title, "RigPilot")
      |> assign(:procs, ManagedProc.all())
      |> assign(:tele, tele)
      |> assign(:history, seed_history(tele))
      |> assign(:estop, false)
      |> assign(:confirm_deploy, false)

    {:ok, socket}
  end

  # ── PubSub handlers ───────────────────────────────────────────────────────

  @impl true
  def handle_info(:procs_changed, socket) do
    {:noreply, assign(socket, :procs, ManagedProc.all())}
  end

  def handle_info({:telemetry, tele}, socket) do
    {:noreply,
     socket
     |> assign(:tele, tele)
     |> update(:history, &push_history(&1, tele))}
  end

  def handle_info(_msg, socket), do: {:noreply, socket}

  # ── UI events (all hit PLACEHOLDER backends) ──────────────────────────────

  @impl true
  def handle_event("start", %{"name" => name}, socket) do
    name = String.to_existing_atom(name)

    # Gate the deploy: open the confirm modal instead of starting immediately.
    if name == :deploy do
      {:noreply, assign(socket, :confirm_deploy, true)}
    else
      ManagedProc.start(name)
      {:noreply, assign(socket, :procs, ManagedProc.all())}
    end
  end

  def handle_event("stop", %{"name" => name}, socket) do
    ManagedProc.stop(String.to_existing_atom(name))
    {:noreply, assign(socket, :procs, ManagedProc.all())}
  end

  def handle_event("restart", %{"name" => name}, socket) do
    ManagedProc.restart(String.to_existing_atom(name))
    {:noreply, assign(socket, :procs, ManagedProc.all())}
  end

  def handle_event("confirm_deploy", _params, socket) do
    ManagedProc.start(:deploy)

    {:noreply,
     socket
     |> assign(:confirm_deploy, false)
     |> assign(:procs, ManagedProc.all())}
  end

  def handle_event("cancel_deploy", _params, socket) do
    {:noreply, assign(socket, :confirm_deploy, false)}
  end

  def handle_event("estop", _params, socket) do
    # PLACEHOLDER E-STOP — would cut rt/lowcmd within one tick in Phase 2.
    ManagedProc.stop(:deploy)

    {:noreply,
     socket
     |> assign(:estop, true)
     |> assign(:procs, ManagedProc.all())}
  end

  def handle_event("clear_estop", _params, socket) do
    {:noreply, assign(socket, :estop, false)}
  end

  # ── render ────────────────────────────────────────────────────────────────

  @impl true
  def render(assigns) do
    ~H"""
    <div class="min-h-screen bg-slate-950 text-slate-100">
      <%= if @estop do %>
        <div class="bg-red-600 text-white font-bold tracking-wide text-center py-2 animate-pulse flex items-center justify-center gap-4">
          <span>⛔ E-STOP ENGAGED — deploy commanded to stop (placeholder). rt/lowcmd would be cut.</span>
          <button
            phx-click="clear_estop"
            class="rounded bg-white/20 hover:bg-white/30 px-3 py-0.5 text-sm"
          >
            Clear
          </button>
        </div>
      <% end %>

      <header class="border-b border-slate-800 bg-slate-900/60 backdrop-blur">
        <div class="mx-auto max-w-7xl px-6 py-4 flex items-center justify-between">
          <div class="flex items-center gap-3">
            <div class="text-2xl font-black tracking-tight">
              Rig<span class="text-emerald-400">Pilot</span>
            </div>
            <span class="rounded-full bg-amber-500/15 text-amber-300 text-xs font-semibold px-2.5 py-1 border border-amber-500/30">
              PHASE 1 · PLACEHOLDER (no live rig contact)
            </span>
          </div>
          <div class="flex items-center gap-4">
            <span class="text-xs text-slate-400">DGX Spark + Unitree G1 teleop rig</span>
            <button
              phx-click="estop"
              class="rounded-lg bg-red-600 hover:bg-red-500 active:bg-red-700 text-white font-extrabold text-lg px-6 py-2.5 shadow-lg shadow-red-900/50 ring-2 ring-red-400/40 transition"
            >
              ⛔ E-STOP
            </button>
          </div>
        </div>
      </header>

      <main class="mx-auto max-w-7xl px-6 py-6 space-y-6">
        <.pipeline procs={@procs} tele={@tele} />

        <div class="grid grid-cols-1 lg:grid-cols-3 gap-6">
          <section class="lg:col-span-2">
            <h2 class="text-sm font-semibold uppercase tracking-wider text-slate-400 mb-3">
              Processes
            </h2>
            <div class="grid grid-cols-1 sm:grid-cols-2 gap-4">
              <.proc_card :for={name <- order()} proc={@procs[name]} />
            </div>
          </section>

          <section>
            <h2 class="text-sm font-semibold uppercase tracking-wider text-slate-400 mb-3">
              Telemetry
            </h2>
            <.telemetry_panel tele={@tele} history={@history} />
          </section>
        </div>
      </main>

      <%= if @confirm_deploy do %>
        <.deploy_modal proc={@procs[:deploy]} />
      <% end %>
    </div>
    """
  end

  # ── pipeline graph ────────────────────────────────────────────────────────

  defp pipeline(assigns) do
    assigns = assign(assigns, :hops, pipeline_hops(assigns.procs, assigns.tele))

    ~H"""
    <section class="rounded-xl border border-slate-800 bg-slate-900/50 p-5">
      <h2 class="text-sm font-semibold uppercase tracking-wider text-slate-400 mb-4">
        End-to-end pipeline
      </h2>
      <div class="flex items-center gap-1 overflow-x-auto pb-2">
        <%= for {hop, idx} <- Enum.with_index(@hops) do %>
          <%= if idx > 0 do %>
            <div class={["text-2xl shrink-0", arrow_color(Enum.at(@hops, idx - 1).health)]}>
              →
            </div>
          <% end %>
          <div class={[
            "shrink-0 rounded-lg border px-3 py-2 text-center min-w-[88px]",
            hop_classes(hop.health)
          ]}>
            <div class="text-xs font-semibold">{hop.label}</div>
            <div class="text-[10px] opacity-80 mt-0.5">{hop.detail}</div>
          </div>
        <% end %>
      </div>
    </section>
    """
  end

  # ── per-process card ──────────────────────────────────────────────────────

  defp proc_card(assigns) do
    ~H"""
    <div class="rounded-xl border border-slate-800 bg-slate-900/50 p-4 flex flex-col gap-3">
      <div class="flex items-start justify-between gap-2">
        <div>
          <div class="font-semibold text-slate-100 leading-tight">{@proc.label}</div>
          <div class="text-xs text-slate-500 mt-0.5">{@proc.host}</div>
        </div>
        <.status_badge status={@proc.status} />
      </div>

      <div class="flex items-center gap-4 text-xs text-slate-400">
        <span>uptime <span class="text-slate-200 font-mono">{fmt_uptime(@proc)}</span></span>
        <span>restarts <span class="text-slate-200 font-mono">{@proc.restart_count}</span></span>
        <span class="rounded bg-slate-800 px-1.5 py-0.5 text-slate-300 text-[10px] uppercase">
          {@proc.kind}
        </span>
      </div>

      <%= if @proc.name == :deploy do %>
        <div class={[
          "rounded-md px-2.5 py-1.5 text-xs flex items-center gap-2",
          deploy_output_classes(@proc.output_type)
        ]}>
          <span class="font-semibold">--output-type {@proc.output_type}</span>
          <%= if @proc.output_type == "log" do %>
            <span class="font-bold">⚠ DRY-RUN — silently never actuates!</span>
          <% else %>
            <span class="opacity-80">actuates the robot</span>
          <% end %>
        </div>
      <% end %>

      <div class="rounded-md bg-slate-950/80 border border-slate-800 p-2 font-mono text-[10.5px] leading-relaxed text-slate-400 h-[72px] overflow-y-auto">
        <div :for={line <- Enum.take(@proc.recent_logs, -5)}>{line}</div>
        <div :if={@proc.recent_logs == []} class="italic opacity-50">no log lines</div>
      </div>

      <div class="flex items-center gap-2 pt-1">
        <button
          phx-click="start"
          phx-value-name={@proc.name}
          disabled={@proc.status in [:running, :starting]}
          class={[
            "rounded-md px-3 py-1.5 text-xs font-semibold transition",
            start_btn_classes(@proc)
          ]}
        >
          <%= if @proc.gated, do: "Start ⚠", else: "Start" %>
        </button>
        <button
          phx-click="stop"
          phx-value-name={@proc.name}
          disabled={@proc.status in [:down, :stopped]}
          class="rounded-md px-3 py-1.5 text-xs font-semibold bg-slate-700 hover:bg-slate-600 disabled:opacity-40 disabled:cursor-not-allowed transition"
        >
          Stop
        </button>
        <button
          phx-click="restart"
          phx-value-name={@proc.name}
          class="rounded-md px-3 py-1.5 text-xs font-semibold bg-slate-800 hover:bg-slate-700 transition"
        >
          Restart
        </button>
      </div>
    </div>
    """
  end

  defp status_badge(assigns) do
    ~H"""
    <span class={[
      "inline-flex items-center gap-1.5 rounded-full px-2.5 py-1 text-xs font-bold whitespace-nowrap",
      status_classes(@status)
    ]}>
      <span class={["h-2 w-2 rounded-full", dot_classes(@status)]}></span>
      {status_label(@status)}
    </span>
    """
  end

  # ── deploy confirm modal ──────────────────────────────────────────────────

  defp deploy_modal(assigns) do
    ~H"""
    <div class="fixed inset-0 z-50 flex items-center justify-center bg-black/70 p-4">
      <div class="w-full max-w-md rounded-xl border border-red-500/40 bg-slate-900 p-6 shadow-2xl">
        <div class="flex items-center gap-2 text-red-400 text-lg font-bold mb-3">
          ⚠ Start deploy — actuates the real robot
        </div>
        <ul class="space-y-1.5 text-sm text-slate-300 mb-4 list-disc list-inside">
          <li>Robot secured on the gantry?</li>
          <li>Workspace clear of people and obstacles?</li>
          <li>Hand on the physical e-stop?</li>
        </ul>
        <div class={[
          "rounded-md px-3 py-2 text-sm mb-5",
          deploy_output_classes(@proc.output_type)
        ]}>
          Resolved output: <span class="font-bold">--output-type {@proc.output_type}</span>
          <%= if @proc.output_type == "log" do %>
            — <span class="font-bold">DRY-RUN, will NOT actuate.</span>
          <% else %>
            — will drive <code>rt/lowcmd</code>.
          <% end %>
        </div>
        <div class="flex justify-end gap-3">
          <button
            phx-click="cancel_deploy"
            class="rounded-md px-4 py-2 text-sm font-semibold bg-slate-700 hover:bg-slate-600"
          >
            Cancel
          </button>
          <button
            phx-click="confirm_deploy"
            class="rounded-md px-4 py-2 text-sm font-bold bg-red-600 hover:bg-red-500 text-white"
          >
            Confirm &amp; Start
          </button>
        </div>
        <p class="text-[10px] text-slate-500 mt-4 text-center">
          Placeholder — flips state only, nothing is actuated.
        </p>
      </div>
    </div>
    """
  end

  # ── telemetry panel ───────────────────────────────────────────────────────

  defp telemetry_panel(assigns) do
    ~H"""
    <div class="space-y-4">
      <div class="rounded-xl border border-slate-800 bg-slate-900/50 p-4 space-y-4">
        <.spark_tile
          label="lowstate Hz"
          value={@tele.dds.lowstate_hz}
          unit="Hz"
          points={@history.lowstate_hz}
          color="#34d399"
          ok={@tele.dds.lowstate_hz > 0}
        />
        <.spark_tile
          label="lowcmd Hz"
          value={@tele.dds.lowcmd_hz}
          unit="Hz"
          points={@history.lowcmd_hz}
          color="#60a5fa"
          ok={@tele.dds.lowcmd_hz > 0}
        />
        <.spark_tile
          label="PICO fps"
          value={@tele.pico.fps}
          unit="fps"
          points={@history.fps}
          color="#fbbf24"
          ok={@tele.pico.fps > 40}
        />
      </div>

      <div class="rounded-xl border border-slate-800 bg-slate-900/50 p-4 space-y-3">
        <div class="flex items-center justify-between">
          <span class="text-sm text-slate-300">PICO body feed</span>
          <span class={[
            "rounded-full px-2.5 py-1 text-xs font-bold",
            if(@tele.pico.pose_chg,
              do: "bg-emerald-500/15 text-emerald-300 border border-emerald-500/30",
              else: "bg-red-500/15 text-red-300 border border-red-500/30"
            )
          ]}>
            <%= if @tele.pico.pose_chg, do: "LIVE", else: "STALE" %>
          </span>
        </div>
        <p class="text-[10px] text-slate-500 -mt-1">
          keyed on <code>pose_chg</code> (body_ts_adv ignored — known PICO bug)
        </p>

        <.kv label="PICO connected" value={yes_no(@tele.pico.connected)} ok={@tele.pico.connected} />
        <.kv label="PICO miss" value={@tele.pico.miss} ok={@tele.pico.miss == 0} />
        <.kv
          label="zenoh sessions"
          value={@tele.zenoh.spark_to_orin_sessions}
          ok={@tele.zenoh.spark_to_orin_sessions > 0}
        />
        <.kv
          label="secondary IMU Hz"
          value={@tele.dds.secondary_imu_hz}
          ok={@tele.dds.secondary_imu_hz > 0}
        />
        <.kv
          label="ping .161 (robot)"
          value={"#{@tele.net.robot_161_ping_ms} ms"}
          ok={@tele.net.robot_161_ping_ms > 0}
        />
        <.kv
          label="ping .164 (Orin)"
          value={"#{@tele.net.robot_164_ping_ms} ms"}
          ok={@tele.net.robot_164_ping_ms > 0}
        />
        <.kv
          label="deploy output"
          value={@tele.deploy.output_type}
          ok={@tele.deploy.output_type != "log"}
        />
      </div>
    </div>
    """
  end

  defp spark_tile(assigns) do
    ~H"""
    <div>
      <div class="flex items-baseline justify-between">
        <span class="text-xs uppercase tracking-wide text-slate-400">{@label}</span>
        <span class={["font-mono text-lg font-bold", if(@ok, do: "text-slate-100", else: "text-red-400")]}>
          {@value}<span class="text-xs text-slate-500 ml-0.5">{@unit}</span>
        </span>
      </div>
      <div class="mt-1">{sparkline(@points, @color)}</div>
    </div>
    """
  end

  defp kv(assigns) do
    ~H"""
    <div class="flex items-center justify-between text-sm">
      <span class="text-slate-400">{@label}</span>
      <span class={["font-mono font-semibold", if(@ok, do: "text-emerald-300", else: "text-amber-300")]}>
        {@value}
      </span>
    </div>
    """
  end

  # ── inline SVG sparkline (no JS deps) ─────────────────────────────────────

  defp sparkline([], _color), do: Phoenix.HTML.raw("")

  defp sparkline(points, color) do
    w = 240
    h = 36
    n = length(points)
    lo = Enum.min(points)
    hi = Enum.max(points)
    span = if hi - lo < 1.0e-6, do: 1.0, else: hi - lo

    coords =
      points
      |> Enum.with_index()
      |> Enum.map(fn {v, i} ->
        x = if n <= 1, do: 0.0, else: i / (n - 1) * w
        y = h - (v - lo) / span * (h - 4) - 2
        "#{Float.round(x * 1.0, 1)},#{Float.round(y * 1.0, 1)}"
      end)
      |> Enum.join(" ")

    last = List.last(points)
    last_x = w * 1.0
    last_y = h - (last - lo) / span * (h - 4) - 2

    Phoenix.HTML.raw("""
    <svg viewBox="0 0 #{w} #{h}" preserveAspectRatio="none" class="w-full h-9">
      <polyline fill="none" stroke="#{color}" stroke-width="1.5"
                stroke-linejoin="round" stroke-linecap="round" points="#{coords}" />
      <circle cx="#{Float.round(last_x * 1.0, 1)}" cy="#{Float.round(last_y * 1.0, 1)}" r="2.2" fill="#{color}" />
    </svg>
    """)
  end

  # ── pipeline health derivation ────────────────────────────────────────────

  defp pipeline_hops(procs, tele) do
    [
      hop("PICO", pico_health(tele), pico_detail(tele)),
      hop("service", proc_health(procs[:roboticsservice]), short_status(procs[:roboticsservice])),
      hop("manager", proc_health(procs[:pico_manager]), short_status(procs[:pico_manager])),
      hop("ZMQ", zmq_health(procs, tele), ":5556"),
      hop("deploy", deploy_pipeline_health(procs[:deploy], tele), deploy_pipeline_detail(procs[:deploy], tele)),
      hop("zenoh", proc_health(procs[:zenoh_spark_bridge]), zenoh_detail(tele)),
      hop("Orin", proc_health(procs[:orin_bridge]), short_status(procs[:orin_bridge])),
      hop("robot", robot_health(tele), "#{tele.dds.lowstate_hz} Hz")
    ]
  end

  defp hop(label, health, detail), do: %{label: label, health: health, detail: detail}

  defp proc_health(nil), do: :red
  defp proc_health(%{status: :running}), do: :green
  defp proc_health(%{status: :starting}), do: :amber
  defp proc_health(%{status: :wedged}), do: :amber
  defp proc_health(_), do: :red

  defp pico_health(%{pico: %{connected: false}}), do: :red
  defp pico_health(%{pico: %{pose_chg: true}}), do: :green
  defp pico_health(_), do: :amber

  defp pico_detail(%{pico: p}), do: if(p.pose_chg, do: "LIVE", else: "STALE")

  defp zmq_health(procs, tele) do
    case {proc_health(procs[:pico_manager]), tele.pico.pose_chg} do
      {:green, true} -> :green
      {:green, false} -> :amber
      {h, _} -> h
    end
  end

  defp deploy_pipeline_health(nil, _), do: :red

  defp deploy_pipeline_health(%{status: :running} = d, tele) do
    cond do
      d.output_type == "log" -> :amber
      tele.deploy.actuating -> :green
      true -> :amber
    end
  end

  defp deploy_pipeline_health(%{status: :starting}, _), do: :amber
  defp deploy_pipeline_health(_, _), do: :red

  defp deploy_pipeline_detail(%{status: :running, output_type: "log"}, _), do: "dry-run"
  defp deploy_pipeline_detail(%{status: :running}, _), do: "actuating"
  defp deploy_pipeline_detail(d, _), do: short_status(d)

  defp zenoh_detail(tele), do: "#{tele.zenoh.spark_to_orin_sessions} sess"

  defp robot_health(%{dds: %{lowstate_hz: hz}}) when hz > 0, do: :green
  defp robot_health(_), do: :red

  defp short_status(nil), do: "?"
  defp short_status(%{status: s}), do: status_label(s)

  # ── history / sparkline buffers ───────────────────────────────────────────

  defp seed_history(tele) do
    %{
      lowstate_hz: [tele.dds.lowstate_hz],
      lowcmd_hz: [tele.dds.lowcmd_hz],
      fps: [tele.pico.fps]
    }
  end

  defp push_history(hist, tele) do
    %{
      lowstate_hz: cap([tele.dds.lowstate_hz | Enum.reverse(hist.lowstate_hz)]),
      lowcmd_hz: cap([tele.dds.lowcmd_hz | Enum.reverse(hist.lowcmd_hz)]),
      fps: cap([tele.pico.fps | Enum.reverse(hist.fps)])
    }
  end

  # keeps newest-last ordering, capped to @history
  defp cap(newest_first), do: newest_first |> Enum.take(@history) |> Enum.reverse()

  # ── presentation helpers ──────────────────────────────────────────────────

  defp order, do: [:roboticsservice, :pico_manager, :zenoh_spark_bridge, :orin_bridge, :deploy, :sim]

  defp status_label(:down), do: "DOWN"
  defp status_label(:starting), do: "STARTING"
  defp status_label(:running), do: "RUNNING"
  defp status_label(:wedged), do: "WEDGED"
  defp status_label(:stopped), do: "STOPPED"

  defp status_classes(:running), do: "bg-emerald-500/15 text-emerald-300 border border-emerald-500/30"
  defp status_classes(:starting), do: "bg-sky-500/15 text-sky-300 border border-sky-500/30"
  defp status_classes(:wedged), do: "bg-amber-500/15 text-amber-300 border border-amber-500/30"
  defp status_classes(:stopped), do: "bg-slate-600/20 text-slate-300 border border-slate-600/40"
  defp status_classes(:down), do: "bg-red-500/15 text-red-300 border border-red-500/30"

  defp dot_classes(:running), do: "bg-emerald-400"
  defp dot_classes(:starting), do: "bg-sky-400 animate-pulse"
  defp dot_classes(:wedged), do: "bg-amber-400 animate-pulse"
  defp dot_classes(:stopped), do: "bg-slate-400"
  defp dot_classes(:down), do: "bg-red-400"

  defp hop_classes(:green), do: "border-emerald-500/40 bg-emerald-500/10 text-emerald-200"
  defp hop_classes(:amber), do: "border-amber-500/40 bg-amber-500/10 text-amber-200"
  defp hop_classes(:red), do: "border-red-500/40 bg-red-500/10 text-red-200"

  defp arrow_color(:green), do: "text-emerald-500"
  defp arrow_color(:amber), do: "text-amber-500"
  defp arrow_color(:red), do: "text-red-500/60"

  defp start_btn_classes(%{gated: true}),
    do: "bg-amber-600 hover:bg-amber-500 text-white disabled:opacity-40 disabled:cursor-not-allowed"

  defp start_btn_classes(_),
    do: "bg-emerald-600 hover:bg-emerald-500 text-white disabled:opacity-40 disabled:cursor-not-allowed"

  defp deploy_output_classes("log"), do: "bg-amber-500/15 text-amber-200 border border-amber-500/40"
  defp deploy_output_classes(_), do: "bg-slate-800 text-slate-300 border border-slate-700"

  defp fmt_uptime(%{status: :running, uptime: s}) when s >= 0 do
    h = div(s, 3600)
    m = div(rem(s, 3600), 60)
    sec = rem(s, 60)

    cond do
      h > 0 -> "#{h}h #{m}m"
      m > 0 -> "#{m}m #{sec}s"
      true -> "#{sec}s"
    end
  end

  defp fmt_uptime(_), do: "—"

  defp yes_no(true), do: "yes"
  defp yes_no(false), do: "no"
end
