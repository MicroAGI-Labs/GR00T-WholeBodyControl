defmodule RigPilotWeb.DashboardLiveTest do
  use RigPilotWeb.ConnCase, async: false

  import Phoenix.LiveViewTest

  test "renders the dashboard with pipeline, processes and E-STOP", %{conn: conn} do
    {:ok, _live, html} = live(conn, "/")
    assert html =~ "RigPilot"
    assert html =~ "E-STOP"
    assert html =~ "End-to-end pipeline"
    assert html =~ "roboticsservice"
    assert html =~ "pico_manager"
    assert html =~ "lowstate Hz"
  end

  test "stop then start a non-gated process flips its state (placeholder)", %{conn: conn} do
    {:ok, live, _html} = live(conn, "/")

    # sim starts stopped; start it -> goes :starting
    render_click(live, "start", %{"name" => "sim"})
    assert RigPilot.ManagedProc.get(:sim).status in [:starting, :running]

    render_click(live, "stop", %{"name" => "sim"})
    assert RigPilot.ManagedProc.get(:sim).status == :stopped
  end

  test "deploy Start is gated behind a confirm modal", %{conn: conn} do
    {:ok, live, _html} = live(conn, "/")

    # ensure deploy is stopped first
    RigPilot.ManagedProc.stop(:deploy)

    # clicking start on deploy opens the modal, does NOT start it
    html = render_click(live, "start", %{"name" => "deploy"})
    assert html =~ "actuates the real robot"
    assert RigPilot.ManagedProc.get(:deploy).status == :stopped

    # confirming actually starts it
    render_click(live, "confirm_deploy", %{})
    assert RigPilot.ManagedProc.get(:deploy).status in [:starting, :running]
  end

  test "E-STOP sets the banner and stops deploy", %{conn: conn} do
    {:ok, live, _html} = live(conn, "/")
    html = render_click(live, "estop", %{})
    assert html =~ "E-STOP ENGAGED"
    assert RigPilot.ManagedProc.get(:deploy).status == :stopped
  end
end
