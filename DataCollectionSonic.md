# Recording a SONIC Dataset on the Real G1

This doc walks through the data‑collection pipeline driven from **this repo**
(`GR00T-WholeBodyControl`) on the workstation, talking to a real Unitree G1 over
the Zenoh ↔ DDS bridge. By the end you'll have episodes written to a local
folder under `outputs/` that can be processed into a training dataset.

It assumes:

- The rig is powered and the workstation is on the same LAN as the G1
  (G1 reachable at `192.168.123.164`).
- The PICO headset is charged and running the SONIC teleop Unity app.
- The repo venvs already exist (`.venv_teleop`, `.venv_data_collection`). If
  not, run `bash install_scripts/install_pico.sh` and
  `bash install_scripts/install_data_collection.sh` from the repo root.

For background on the split‑brain Spark ↔ G1 architecture and the Zenoh/DDS
bridge, see `README.md` and `Inference.md`.

---

## 1. Components that must be running

End‑to‑end data collection requires five things alive at the same time, on
different hosts:

| # | Component                              | Where it runs                  | How it gets started                                       |
|---|----------------------------------------|--------------------------------|-----------------------------------------------------------|
| 1 | **Camera server (ZMQ :5555)**          | G1, in Docker                  | Autostarts on boot; verify with `docker ps`               |
| 2 | **Zenoh ↔ DDS bridge**                 | G1 (`unitree@192.168.123.164`) | `~/zenoh-dds/start_bridge.sh` on the G1                   |
| 3 | **SONIC C++ deploy (`g1zenoh`)**       | Workstation                    | `gear_sonic_deploy/deploy.sh` (this repo)                 |
| 4 | **PICO manager / SMPL pose server**    | Workstation (`.venv_teleop`)   | `gear_sonic/scripts/pico_manager_thread_server.py`        |
| 5 | **Data exporter**                      | Workstation (`.venv_data_collection`) | `gear_sonic/scripts/run_data_exporter.py`          |

The PICO headset app connects to (4) over the network and streams body / hand
tracking; (3) consumes those poses via ZMQ and drives the robot; (5) subscribes
to the robot state, the SMPL pose, and the camera and writes episodes to disk.

> Order matters. Bring them up roughly in the order above — the exporter (5)
> will refuse to start until it has received a `robot_config` message from the
> C++ deploy (3).

---

## 2. Verify the camera server on the G1

The camera feed is published by a Docker container on the G1 that autostarts on
boot, so you normally don't need to touch it — just confirm it's healthy:

```bash
ssh unitree@192.168.123.164 'docker ps --filter name=gear-camera --format "{{.Status}}"'
```

Expected output: a single `Up …` line (e.g. `Up 3 hours (healthy)`). If
nothing is printed, the container isn't running and the exporter will sit
without frames. See `Inference.md` §4 and the camera‑server docs for how to
bring it back up.

You can also confirm that frames actually flow into the workstation:

```bash
.venv_sim/bin/python gear_sonic/scripts/run_camera_viewer.py \
    --camera-host 192.168.123.164 \
    --camera-port 5555
```

A live image window should appear. Close it once you're satisfied.

---

## 3. Start the Zenoh ↔ DDS bridge on the G1

The workstation talks to the G1 over Zenoh; the G1 side bridges Zenoh into the
Unitree DDS domain. Start the bridge on the G1 (the script is already installed
there):

```bash
ssh unitree@192.168.123.164 '~/zenoh-dds/start_bridge.sh'
```

Leave that SSH session open (or run it inside `tmux` / `screen` on the G1) for
as long as you want to collect data. `deploy.sh g1zenoh` (next step) also
starts the Spark‑side bridge container automatically.

---

## 4. Deploy SONIC on the G1 (`g1zenoh` path)

From the **`gear_sonic_deploy/`** directory of this repo, launch the C++
deploy with the ZMQ manager input and `all` output handlers (this is what
publishes both the robot debug stream the exporter reads from, and the DDS
commands the robot consumes):

```bash
cd gear_sonic_deploy
DEPLOY_YES=1 ./deploy.sh --input-type zmq_manager --output-type all g1zenoh
```

Flags:

- `g1zenoh` — selects the Spark‑Orin DDS‑over‑Zenoh deploy path (real G1 via
  the bridge from step 3). The script also launches the Spark‑side
  `zenoh-spark-bridge` Docker container if it isn't already running.
- `--input-type zmq_manager` — the deploy listens for SMPL pose / control
  inputs over ZMQ from the PICO manager (step 5), instead of `keyboard` or the
  Isaac‑GR00T VLA client.
- `--output-type all` — publishes the `g1_debug` and `robot_config` ZMQ topics
  the data exporter subscribes to, in addition to sending DDS commands to the
  robot.
- `DEPLOY_YES=1` — skips the interactive "Proceed? [Y/n]" prompt. Useful under
  shells (e.g. ble.sh) where bash `read` misbehaves.

Leave this terminal running. Once you see the deploy printing periodic
`robot_config` / `g1_debug` lines you can move on.

---

## 5. Start the PICO manager / SMPL pose server

In a new terminal, from the **repo root**:

```bash
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager
```

What it does:

- Receives controller / body‑tracking packets from the PICO headset over the
  network.
- Re‑publishes the resulting SMPL pose on ZMQ port `5556` (topic `pose`), which
  the C++ deploy (step 4) and the data exporter (step 6) both subscribe to.
- The `--manager` flag is what wires it into the deploy's `zmq_manager` input
  type — without it, the C++ side won't receive teleop commands.

Now put the headset on and start the SONIC teleop app on the PICO. Confirm in
the manager's console output that you see body / hand frames coming in and
that the headset reports itself as connected before continuing. The robot
should also begin tracking the operator's pose at this point — make sure the
G1 is on its stand or otherwise safe.

> See `memory: pico-teleop-mode-commands` for the controller button combos
> that switch the G1 between modes (locomotion vs. SMPL/full‑body) while the
> manager is running.

---

## 6. Start the data exporter

In a third terminal, from the **repo root**:

```bash
source .venv_data_collection/bin/activate
python gear_sonic/scripts/run_data_exporter.py \
    --task-prompt "pick up the box" \
    --camera-host 192.168.123.164 \
    --camera-port 5555
```

What the flags do:

- `--task-prompt "pick up the box"` — language instruction written into every
  episode's `info.json` as the task label. Change this per recording session.
- `--camera-host 192.168.123.164` / `--camera-port 5555` — the G1's IP and the
  ZMQ camera server port from step 2. Without these, the exporter would try
  `localhost:5555` and fail.

Useful extras (see `run_data_exporter.py --help` for the full list):

- `--dataset-name my_session` — pin a dataset folder name instead of the
  auto‑generated timestamped one.
- `--root-output-dir outputs` — change where episodes are written (default
  `outputs/`).
- `--data-collection-frequency 50` — Hz of the recording loop (default 50).
- `--record-wrist-cameras` — also record the left/right wrist streams.
- `--no-text-to-speech` — disable spoken status feedback.

On startup the exporter waits for a `robot_config` message from the C++ deploy
on ZMQ port `5557` (topic `robot_config`). If you see it block here, double‑
check step 4 is actually running and publishing `--output-type all`.

Once it's recording, episodes are appended under
`outputs/<dataset_name>/` in the LeRobot‑compatible layout consumed by
`gear_sonic/scripts/process_dataset.py`.

---

## 7. Pre‑flight checklist

Before pressing record:

1. `ssh unitree@192.168.123.164 'docker ps --filter name=gear-camera --format "{{.Status}}"'`
   shows an `Up …` line. (§2)
2. `ssh unitree@192.168.123.164 '~/zenoh-dds/start_bridge.sh'` is running. (§3)
3. `gear_sonic_deploy/deploy.sh ... g1zenoh` terminal is printing
   `robot_config` / `g1_debug` lines. (§4)
4. `pico_manager_thread_server.py --manager` terminal shows live frames from
   the headset and the PICO app reports connected. (§5)
5. `run_data_exporter.py` started and reached the recording loop (no
   `robot_config` timeout). (§6)
6. The G1 is on its stand or otherwise in a safe pose, mode is set via the
   controller, and there's a human nearby holding the E‑stop.

If any of (1)–(4) is missing, the exporter will either block at startup or
record episodes with stale / missing state — don't ship those.

---

## 8. Shutting down

Stop in the reverse order you started:

1. `Ctrl-C` the data exporter (step 6) — it will finalise the current episode.
2. `Ctrl-C` the PICO manager (step 5).
3. `Ctrl-C` the deploy (step 4). The Spark‑side `zenoh-spark-bridge` Docker
   container is left running; remove it with
   `docker rm -f zenoh-spark-bridge` if you want a clean slate.
4. Close the SSH session running the G1‑side Zenoh bridge (step 3).
5. The camera server (step 1) can be left running — it autostarts on boot.

---

## 9. Processing the recorded data

Once you have one or more episodes under `outputs/<dataset_name>/`, convert
them into the training‑ready dataset format with:

```bash
source .venv_data_collection/bin/activate
python gear_sonic/scripts/process_dataset.py --help
```

(See `process_dataset.py` for the exact flags — it post‑processes the raw
episodes into the LeRobot/GR00T schema expected by Isaac‑GR00T training.)
