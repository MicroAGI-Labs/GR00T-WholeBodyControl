"""Fire a single reset-all (category 2) after DELAY seconds to release the base-hold."""
import sys, time
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
DELAY = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
CAT = sys.argv[2] if len(sys.argv) > 2 else "2"
ChannelFactoryInitialize(1)
pub = ChannelPublisher("rt/reset_pose/cmd", String_); pub.Init()
time.sleep(DELAY)
m = std_msgs_msg_dds__String_(); m.data = CAT
pub.Write(m)
print(f"reset cat{CAT} fired at +{DELAY}s", flush=True)
time.sleep(1)
