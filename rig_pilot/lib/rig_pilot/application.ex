defmodule RigPilot.Application do
  # See https://hexdocs.pm/elixir/Application.html
  # for more information on OTP Applications
  @moduledoc false

  use Application

  @impl true
  def start(_type, _args) do
    children = [
      RigPilotWeb.Telemetry,
      {DNSCluster, query: Application.get_env(:rig_pilot, :dns_cluster_query) || :ignore},
      {Phoenix.PubSub, name: RigPilot.PubSub},
      # Registry that lets ManagedProc GenServers be addressed by their atom name.
      {Registry, keys: :unique, name: RigPilot.ProcRegistry},
      # PLACEHOLDER rig backends — Phase 1 in-memory state only (no rig contact).
      RigPilot.ProcSupervisor,
      RigPilot.TelemetrySource,
      # Start to serve requests, typically the last entry
      RigPilotWeb.Endpoint
    ]

    # See https://hexdocs.pm/elixir/Supervisor.html
    # for other strategies and supported options
    opts = [strategy: :one_for_one, name: RigPilot.Supervisor]
    Supervisor.start_link(children, opts)
  end

  # Tell Phoenix to update the endpoint configuration
  # whenever the application is updated.
  @impl true
  def config_change(changed, _new, removed) do
    RigPilotWeb.Endpoint.config_change(changed, removed)
    :ok
  end
end
