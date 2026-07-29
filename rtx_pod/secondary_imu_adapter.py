"""Republish the sim G1 LowState IMU as rt/secondary_imu (IMUState_).

The real G1 has a second IMU in the torso published on rt/secondary_imu;
unitree_sim_isaaclab only fills LowState.imu_state. The SONIC deploy requires
both, so mirror the pelvis IMU. Approximation: torso == pelvis (valid near
neutral waist).
"""
import time
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, IMUState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__IMUState_

ChannelFactoryInitialize(1)
pub = ChannelPublisher("rt/secondary_imu", IMUState_)
pub.Init()
count = [0]

def cb(msg):
    imu = unitree_hg_msg_dds__IMUState_()
    src = msg.imu_state
    imu.quaternion = src.quaternion
    imu.gyroscope = src.gyroscope
    imu.accelerometer = src.accelerometer
    imu.rpy = src.rpy
    imu.temperature = src.temperature
    pub.Write(imu)
    count[0] += 1
    if count[0] % 500 == 0:
        print(f"[adapter] relayed {count[0]} imu msgs", flush=True)

sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(cb, 10)
print("[adapter] running", flush=True)
while True:
    time.sleep(5)
