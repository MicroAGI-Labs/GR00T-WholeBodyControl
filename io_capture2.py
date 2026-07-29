"""Rich SONIC I/O capture on a DDS domain -> CSV + live summary.
Captures rt/lowstate (measured q/dq/tau, pelvis IMU quat/gyro/acc) and
rt/lowcmd (commanded q/kp/kd) for all 29 joints at ~50Hz.
Usage: io_capture2.py <domain> <label> <dur> <out.csv> [interface]
"""
import sys, time, math
sys.path.insert(0, "/home/microagi/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_

dom = int(sys.argv[1]); label = sys.argv[2]
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 16.0
OUT = sys.argv[4] if len(sys.argv) > 4 else f"/tmp/cap_{label}.csv"
IFACE = sys.argv[5] if len(sys.argv) > 5 else None
N = 29
if IFACE:
    ChannelFactoryInitialize(dom, IFACE)
else:
    ChannelFactoryInitialize(dom)
st = {"mq": None, "mdq": None, "mtau": None, "cq": None, "ckp": None, "ckd": None,
      "quat": None, "gyro": None, "acc": None, "ls_t": 0.0, "lc_t": 0.0}

def scb(m):
    st["mq"] = [m.motor_state[i].q for i in range(N)]
    st["mdq"] = [m.motor_state[i].dq for i in range(N)]
    st["mtau"] = [m.motor_state[i].tau_est for i in range(N)]
    st["quat"] = list(m.imu_state.quaternion)
    st["gyro"] = list(m.imu_state.gyroscope)
    st["acc"] = list(m.imu_state.accelerometer)
    st["ls_t"] = time.time()
def ccb(m):
    st["cq"] = [m.motor_cmd[i].q for i in range(N)]
    st["ckp"] = [m.motor_cmd[i].kp for i in range(N)]
    st["ckd"] = [m.motor_cmd[i].kd for i in range(N)]
    st["lc_t"] = time.time()

ChannelSubscriber("rt/lowstate", LowState_).Init(scb, 10)
ChannelSubscriber("rt/lowcmd", LowCmd_).Init(ccb, 10)

def rpy(q):
    w, x, y, z = q
    roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y - z*x)))))
    tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2*(x*x + y*y)))))
    return roll, pitch, tilt

idx = {"Lhip_p": 0, "Lknee": 3, "Lank_p": 4, "Rknee": 9, "Rank_p": 10}
hdr = ["t", "tilt", "roll", "pitch", "wnorm", "ls_age_ms", "lc_age_ms"]
hdr += [f"mq{i}" for i in range(N)] + [f"mdq{i}" for i in range(N)]
hdr += [f"mtau{i}" for i in range(N)] + [f"cq{i}" for i in range(N)]
hdr += [f"ckp{i}" for i in range(N)] + [f"ckd{i}" for i in range(N)]
f = open(OUT, "w"); f.write(",".join(hdr) + "\n")

time.sleep(0.8)
print(f"=== {label} (domain {dom}) -> {OUT} ===", flush=True)
print(f"{'t':>5} {'tilt':>6} {'roll':>6} {'pitch':>6} {'|w|':>5} {'lsAge':>6} | "
      + " ".join(f"{k}" for k in idx), flush=True)
t0 = time.time()
last_print = 0.0
while time.time() - t0 < DUR:
    now = time.time(); t = now - t0
    mq, cq, q, g = st["mq"], st["cq"], st["quat"], st["gyro"]
    if mq and cq and q:
        roll, pitch, tilt = rpy(q)
        wn = math.sqrt(sum(v*v for v in g)) if g else 0.0
        ls_age = (now - st["ls_t"]) * 1000; lc_age = (now - st["lc_t"]) * 1000
        row = [f"{t:.3f}", f"{tilt:.2f}", f"{roll:.2f}", f"{pitch:.2f}",
               f"{wn:.3f}", f"{ls_age:.1f}", f"{lc_age:.1f}"]
        row += [f"{v:.4f}" for v in st["mq"]] + [f"{v:.4f}" for v in st["mdq"]]
        row += [f"{v:.4f}" for v in st["mtau"]] + [f"{v:.4f}" for v in st["cq"]]
        row += [f"{v:.2f}" for v in st["ckp"]] + [f"{v:.2f}" for v in st["ckd"]]
        f.write(",".join(row) + "\n")
        if t - last_print >= 0.4:
            last_print = t
            js = " ".join(f"{mq[i]:+.2f}/{cq[i]:+.2f}" for i in idx.values())
            print(f"{t:5.1f} {tilt:6.1f} {roll:6.1f} {pitch:6.1f} {wn:5.2f} "
                  f"{ls_age:6.1f} | {js}", flush=True)
    time.sleep(0.02)
f.close()
print(f"[done] wrote {OUT}", flush=True)
