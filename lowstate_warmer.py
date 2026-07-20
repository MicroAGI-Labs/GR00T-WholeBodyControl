"""Persistent rt/lowstate subscriber to keep the zenoh<->DDS route warm.
Usage: lowstate_warmer.py [domain]  (default 0)"""
import time, sys
sys.path.insert(0, "/home/microagi/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
dom = int(sys.argv[1]) if len(sys.argv) > 1 else 0
ChannelFactoryInitialize(dom)
n = [0]
def cb(m): n[0] += 1
sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(cb, 10)
print(f"[warmer] domain={dom} holding rt/lowstate route warm", flush=True)
last = 0
while True:
    time.sleep(2.0)
    print(f"[warmer d{dom}] lowstate total={n[0]} (+{n[0]-last} in 2s)", flush=True)
    last = n[0]
