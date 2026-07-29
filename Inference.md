# Running GR00T Inference on the Real G1

This doc describes how to run a trained Isaac‑GR00T VLA policy on the real Unitree
G1 hardware from this repo (`GR00T-WholeBodyControl`). It assumes the rig is
already powered, the PC is on the same LAN as the G1 (G1 reachable at
`192.168.123.164`), and an Isaac‑GR00T checkpoint is available to serve.

For background on the split‑brain Spark↔G1 architecture and the Zenoh/DDS
bridge, see `README.md` and the docs under `docs/`.

---

## 1. Components that must be running

End‑to‑end inference requires four things alive at the same time, on different
hosts:

| # | Component                         | Where it runs        | How it gets started                                    |
|---|-----------------------------------|----------------------|--------------------------------------------------------|
| 1 | **Isaac‑GR00T PolicyServer**      | Workstation / GPU box| From the **Isaac‑GR00T** repo (separate terminal)      |
| 2 | **Zenoh ↔ DDS bridge**            | G1 (`unitree@192.168.123.164`) | `~/zenoh-dds/start_bridge.sh` on the G1     |
| 3 | **Camera server (ZMQ :5555)**     | G1, in Docker        | Autostarts on boot; verify with `docker ps`            |
| 4 | **SONIC inference launcher**      | Workstation          | `gear_sonic/scripts/launch_inference.py` (this repo)   |

The launcher in (4) brings up the C++ deploy + VLA client + (optional) data
exporter in a single tmux session, but it does **not** start the policy server,
the Zenoh bridge, or the camera server — those are external dependencies you
need to bring up first.

---

## 2. Start the GR00T policy server (Isaac‑GR00T repo)

The trained VLA policy is served by Isaac‑GR00T's `PolicyServer`, which lives
in a **separate repo** (`Isaac-GR00T`). Start it in its own terminal with the
checkpoint you want to evaluate. It must be reachable from this workstation on
the host/port you'll pass to `launch_inference.py` (defaults: `localhost:5550`).

See the Isaac‑GR00T repo for the exact command and checkpoint flags. Once it's
up, you should see it listening on `5550` (or whichever port you configured).

---

## 3. Bring up the Zenoh ↔ DDS bridge on the G1

The workstation talks to the G1 over Zenoh; the G1 side bridges Zenoh into the
Unitree DDS domain. Start the bridge on the G1 (script is already installed
there):

```bash
ssh unitree@192.168.123.164 '~/zenoh-dds/start_bridge.sh'
```

Leave that SSH session open (or run it inside `tmux` / `screen` on the G1) for
as long as you want to run inference.

---

## 4. Verify the camera server is running on the G1

The camera feed is published by a Docker container on the G1 that autostarts on
boot, so you normally don't need to touch it — just confirm it's healthy:

```bash
ssh unitree@192.168.123.164 'docker ps --filter name=gear-camera --format "{{.Status}}"'
```

Expected output: a single `Up …` line (e.g. `Up 3 hours (healthy)`). If
nothing is printed, the container isn't running and you'll need to start it
before inference will produce useful actions. (See the camera‑server docs for
how to bring it back up.)

You can also confirm that frames are actually flowing into the workstation by
running the viewer from this repo:

```bash
.venv_sim/bin/python gear_sonic/scripts/run_camera_viewer.py \
    --camera-host 192.168.123.164 \
    --camera-port 5555
```

A live image window should appear. Close it once you're satisfied.

---

## 5. Launch inference on the workstation

From the repo root:

```bash
.venv_inference/bin/python gear_sonic/scripts/launch_inference.py \
    --deploy-mode g1zenoh \
    --policy-host localhost --policy-port 5550 \
    --camera-host 192.168.123.164 \
    --prompt "pick up the box from the table and place it in the big box" \
    --no-data-exporter
```

What the flags do:

- `--deploy-mode g1zenoh` — selects the Spark‑Orin DDS‑over‑Zenoh deploy path
  (i.e. real G1 via the bridge you started in step 3). The default would derive
  `sim`/`real` from `--sim`; passing `g1zenoh` explicitly is what we want here.
- `--policy-host` / `--policy-port` — where to reach the Isaac‑GR00T
  `PolicyServer` you started in step 2.
- `--camera-host 192.168.123.164` — the G1's IP, where the ZMQ camera server
  from step 4 is publishing on port `5555` (default).
- `--prompt "…"` — the language instruction sent to the VLA.
- `--no-data-exporter` — skips spawning the recording pane. Drop this flag if
  you also want to log a dataset during the run (requires `.venv_data_collection`).

The launcher will open a tmux session named `sonic_inference` with the C++
deploy, the VLA inference client, a keyboard publisher, and (unless
`--no-data-exporter`) the data exporter. Attach with:

```bash
tmux attach -t sonic_inference
```

Kill the whole session with `Ctrl-\` (bound at session creation) or
`tmux kill-session -t sonic_inference`.

---

## 6. Quick pre‑flight checklist

Before launching:

1. Isaac‑GR00T `PolicyServer` is up on the chosen host:port.
2. `ssh unitree@192.168.123.164 '~/zenoh-dds/start_bridge.sh'` is running.
3. `ssh unitree@192.168.123.164 'docker ps --filter name=gear-camera --format "{{.Status}}"'`
   shows an `Up …` line.
4. (Optional) Camera viewer shows live frames from `192.168.123.164:5555`.
5. Run the `launch_inference.py` command above from the repo root.

If any of (1)–(3) is missing, the launcher will start but the policy won't
produce sensible actions on the robot.
