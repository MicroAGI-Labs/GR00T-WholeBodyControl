"""Publish one simulator lifecycle request.

Usage: fire_reset.py [discovery-delay] [category] [topic-prefix]

For the concurrent MVP, category 2 or 4 re-arms one robot and category 3
releases it after that robot has supplied a fresh LowCmd.  The DDS domain and
interface may be set with DDS_DOMAIN and DDS_INTERFACE.
"""
import os
import sys
import time
import unitree_sdk2py.core.channel as channel_module
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

# The SDK's built-in interface configuration only probes participant indices
# 0..9.  Eight SONIC processes plus the Spark bridge exhaust that range, making
# this one-shot lifecycle publisher fail before it can send anything.  Extend
# the SDK configuration in this process to match the concurrent deploy config.
max_participant_index = int(os.environ.get("DDS_MAX_AUTO_PARTICIPANT_INDEX", "99"))
discovery_xml = (
    "<Discovery><ParticipantIndex>auto</ParticipantIndex>"
    f"<MaxAutoParticipantIndex>{max_participant_index}</MaxAutoParticipantIndex>"
    "</Discovery>"
)
for config_name in ("ChannelConfigHasInterface", "ChannelConfigAutoDetermine"):
    config = getattr(channel_module, config_name)
    if "MaxAutoParticipantIndex" not in config:
        setattr(
            channel_module,
            config_name,
            config.replace("</General>", f"</General>{discovery_xml}", 1),
        )

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
