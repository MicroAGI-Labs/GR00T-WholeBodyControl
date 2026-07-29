"""Capture SONIC controller I/O on a given DDS domain: rt/lowstate (obs in:
measured joints + pelvis IMU) and rt/lowcmd (cmd out: commanded joints).
Usage: io_capture.py <domain> <label> <dur>
Reports base tilt, |base gyro|, and knee/ankle/hip measured-vs-commanded."""
import sys, time, math
sys.path.insert(0, "/home/microagi/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_

dom = int(sys.argv[1]); label = sys.argv[2]; DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
ChannelFactoryInitialize(dom, "lo")
st = {"meas": None, "cmd": None, "quat": None, "gyro": None}
def scb(m):
    st["meas"] = [m.motor_state[i].q for i in range(29)]
    st["quat"] = list(m.imu_state.quaternion); st["gyro"] = list(m.imu_state.gyroscope)
def ccb(m):
    st["cmd"] = [m.motor_cmd[i].q for i in range(29)]
ChannelSubscriber("rt/lowstate", LowState_).Init(scb, 10)
ChannelSubscriber("rt/lowcmd", LowCmd_).Init(ccb, 10)

def tilt(q):
    return math.degrees(math.acos(max(-1, min(1, 1 - 2*(q[1]*q[1]+q[2]*q[2])))))

idx = {"Lhip_p":0,"Lknee":3,"Lank_p":4,"Rknee":9,"Rank_p":10}
time.sleep(1.0)
print(f"=== {label} (domain {dom}) : measured / commanded ===")
print(f"{'t':>4} {'tilt':>5} {'|w|':>5} | " + " ".join(f"{k}" for k in idx))
t0 = time.time()
while time.time() - t0 < DUR:
    m, c, q, g = st["meas"], st["cmd"], st["quat"], st["gyro"]
    if m and c and q:
        wn = math.sqrt(sum(x*x for x in g)) if g else 0
        js = " ".join(f"{m[i]:+.2f}/{c[i]:+.2f}" for i in idx.values())
        print(f"{time.time()-t0:4.1f} {tilt(q):5.1f} {wn:5.2f} | {js}", flush=True)
    else:
        got = f"meas={m is not None} cmd={c is not None} ls={q is not None}"
        print(f"{time.time()-t0:4.1f} waiting ({got})", flush=True)
    time.sleep(0.4)
