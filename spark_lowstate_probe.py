"""Probe rt/lowstate continuity on the Spark's DDS (domain 0). Prints inter-arrival
gaps so we can see whether lowstate is delivered continuously (steady ~20ms) or in
a single burst then stops (which would explain the deploy's 'Lost LowState')."""
import time, sys
sys.path.insert(0, "/home/microagi/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

ChannelFactoryInitialize(0)  # domain 0 (Spark lo, same as deploy)
state = {"n": 0, "last": None, "gaps": [], "first": None}

def cb(m):
    now = time.time()
    state["n"] += 1
    if state["first"] is None:
        state["first"] = now
    if state["last"] is not None:
        state["gaps"].append(now - state["last"])
    state["last"] = now

sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(cb, 10)

t0 = time.time()
while time.time() - t0 < 8.0:
    time.sleep(1.0)
    g = state["gaps"][-50:]
    if g:
        import statistics
        print(f"t={time.time()-t0:.0f}s  count={state['n']}  recent gap avg={1000*statistics.mean(g):.1f}ms max={1000*max(g):.1f}ms", flush=True)
    else:
        print(f"t={time.time()-t0:.0f}s  count={state['n']}  (no messages yet)", flush=True)
print(f"TOTAL lowstate msgs in 8s: {state['n']}  (continuous ~50Hz would be ~400)")
