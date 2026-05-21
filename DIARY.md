# Project Diary: Unitree G1 + GR00T-WholeBodyControl on DGX Spark

## 2026-05-18 — Architecture Decision
- Decided against upgrading the G1 (JetPack 5.1.1 / TensorRT 8.5.2). Too risky for existing low-level control.
- New target: DGX Spark runs modern stack (TensorRT 10.16, SONIC/GEAR-SONIC, PICO service). G1 stays frozen and only executes low-level commands over ZMQ/Ethernet.
- Spark = brain + teleop server. G1 = body.

## 2026-05-18 — G1 Dual-Homed Networking (Layer 1)
- WiFi (Andrews-iPhone-G1) set as primary for internet (metric 100).
- Ethernet kept as reliable local fallback (metric 20100) for motion controller and SSH.
- Careful use of `never-default`, `route-metric`, and clearing stale routes. Autoconnect + Cloudflare DNS (1.1.1.3) enabled.
- Lesson: Always verify with `ip route get 8.8.8.8` and `ip route get 192.168.123.1` after every change.

## 2026-05-18 — GR00T-WholeBodyControl Repo
- Cloned full repo + LFS on the Spark (4.6 GB total).
- Key directories: `gear_sonic_deploy/`, `gear_sonic/`, `motionbricks/`.
- Confirmed G1-specific assets (meshes, USD models, Unitree SDK libs, `roboticsservice_1.0.0.0_arm64.deb`) are present.

## 2026-05-18 — TensorRT 10.16.1 on DGX Spark
- Installed via official Ubuntu 24.04 arm64 local repo (10.16.1 + CUDA 13.2).
- `trtexec` works. C++ libraries and dev packages are solid.
- Python bindings (`python3-libnvinfer`) land in system Python 3.12 only.
- Lesson: Never rely on conda `python3` for system CUDA/TensorRT packages.

## 2026-05-18 — XRoboToolkit-PC-Service (roboticsservice)
- Installed vendored `roboticsservice_1.0.0.0_arm64.deb`.
- Created user systemd service (`~/.config/systemd/user/roboticsservice.service`).
- Hit `libicuuc.so.70` missing → fixed by manually installing `libicu70` from Jammy.
- Service now reaches "release mode".

## 2026-05-18 — PICO Client Setup (XRoboToolkit APK)
- Downloaded XRoboToolkit APK from https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/
- Installed onto PICO headset using `adb install`

## 2026-05-18 — Python Environment Strategy for GR00T
- GR00T uses `uv` + managed Python (mostly 3.10) for `.venv_inference`, `.venv_teleop`, etc.
- On Spark we created `.venv_inference` using **system Python 3.12 + `--system-site-packages`** so TensorRT is visible.
- This is the pattern that works when you have system-installed NVIDIA stacks.

## Key Lessons So Far
- External high-power compute (Spark) + frozen robot (G1) is the right pragmatic split.
- Vendored `.deb`s from PICO/ByteDance are brittle on newer Ubuntu — expect missing libs (icu, etc.).
- Always create project venvs with `--system-site-packages` when depending on system TensorRT/CUDA.
- Network metrics and route verification must be obsessive on dual-homed robots.
- `uv` + system-site-packages is the current winning combo for this hardware mix.
- `adb install` is reliable for getting the XRoboToolkit client onto the PICO.