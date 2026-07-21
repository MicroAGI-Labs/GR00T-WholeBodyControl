"""Publish one simulator lifecycle request.

Usage: fire_reset.py [discovery-delay] [category] [topic-prefix]

For the concurrent MVP, category 2 or 4 re-arms one robot and category 3
releases it after that robot has supplied a fresh LowCmd.  The DDS domain and
interface may be set with DDS_DOMAIN and DDS_INTERFACE.
"""
import os
import sys
import time
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
DELAY = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
CAT = sys.argv[2] if len(sys.argv) > 2 else "2"
PREFIX = (sys.argv[3] if len(sys.argv) > 3 else os.environ.get("SIM_TOPIC_PREFIX", "")).rstrip("/")
DOMAIN = int(os.environ.get("DDS_DOMAIN", "1"))
INTERFACE = os.environ.get("DDS_INTERFACE")
TOPIC = f"{PREFIX}/reset_pose/cmd" if PREFIX else "rt/reset_pose/cmd"
if CAT not in {"1", "2", "3", "4"}:
    raise ValueError(f"invalid reset category: {CAT!r}")
if INTERFACE:
    ChannelFactoryInitialize(DOMAIN, INTERFACE)
else:
    ChannelFactoryInitialize(DOMAIN)
pub = ChannelPublisher(TOPIC, String_); pub.Init()
time.sleep(DELAY)
m = std_msgs_msg_dds__String_(); m.data = CAT
pub.Write(m)
print(
    f"reset cat{CAT} fired on {TOPIC} (domain {DOMAIN}) at +{DELAY:g}s",
    flush=True,
)
time.sleep(1)
