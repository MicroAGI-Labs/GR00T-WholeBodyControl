defmodule RigPilot.ProcSupervisor do
  @moduledoc """
  PLACEHOLDER — wire to muontrap/SSH in Phase 2.

  Supervises one `RigPilot.ManagedProc` GenServer per rig subprocess. In Phase 1
  these children only hold in-memory placeholder state.

  ## Phase 2 wiring point
  Swap this `Supervisor` for a `DynamicSupervisor` with `rest_for_one`-style
  dependency ordering (`roboticsservice → pico_manager`,
  `orin_bridge → zenoh_spark_bridge → deploy`) and exponential backoff /
  restart-storm protection, per §8 of the design doc.
  """

  use Supervisor

  def start_link(_opts) do
    Supervisor.start_link(__MODULE__, :ok, name: __MODULE__)
  end

  @impl true
  def init(:ok) do
    children =
      for name <- RigPilot.ManagedProc.names() do
        Supervisor.child_spec({RigPilot.ManagedProc, name}, id: name)
      end

    Supervisor.init(children, strategy: :one_for_one)
  end
end
