"""stand_eval.py — deterministic in-sim balance evaluator for the flat G1 stand.

Runs INSIDE the Isaac sim loop (it needs the robot's world pose + sim-time,
neither of which is carried on rt/lowstate) and publishes a compact score on the
DDS topic ``rt/eval`` (std_msgs/String JSON, domain 1). The zenoh bridge forwards
it to the Spark exactly like rt/lowstate (add "rt/eval" to the bridge allow-list).

The score is two numbers:

  termination  0..1   1 => the eval is over. It does NOT say pass/fail. It ramps
                      linearly 0->1 across the SUCCESS_WINDOW_S seconds of *sim
                      time* the robot is asked to keep standing (reaching 1 == a
                      full clean stand = success). A fall snaps it to 1 at once.

  result     -100..100   0 is the scale midpoint; +100 = perfect upright stand at
                      the start location; -100 = worst failure (a fall), reserved.
                      While standing it is  100 - dist_pen - tilt_pen -
                      squat_pen - joint_pen  clamped to [-99, 100]:
                        * dist_pen  : leaving the start spot. 0 at 0 m, and >=2 m
                          from start drives the score to -99 (linear in between).
                        * tilt_pen  : leaning off vertical (any direction). 0 up
                          to EVAL_TILT_FREE_DEG (12 deg), then ramps to full weight
                          at the fall tilt.
                        * squat_pen : base dropping below its standing height.
                        * joint_pen : any joint straying outside a reasonably
                          normal (soft-limit) band.

Determinism: every quantity is a function of the sim state and sim-time only
(publish cadence is gated on sim-time, not wall-clock), so a given trajectory
always yields the same eval stream.

Lifecycle from sim_main: arm() at hold-release (controller starts balancing),
disarm() on a re-arm-hold, update() once per control step. It never raises into
the sim loop (all work is guarded); a failure just skips that sample.
"""
import json
import math
import os

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_


def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


class StandEval:
    """Deterministic balance eval → rt/eval. See module docstring."""

    def __init__(self, robot, sim_time_fn, env_id=0, topic="rt/eval"):
        # robot: IsaacLab Articulation (env.scene["robot"]).
        # sim_time_fn: () -> float, monotonic sim-time in seconds.
        self.robot = robot
        self.sim_time = sim_time_fn
        self.env_id = int(env_id)
        self.topic = topic

        # ---- tunables (env-overridable; defaults match the flat-stand recipe) --
        self.window_s = _envf("EVAL_WINDOW_S", 60.0)   # sim-sec standing == success
        self.dist_max = _envf("EVAL_DIST_MAX_M", 2.0)  # >=this from start => -99
        self.fall_tilt = _envf("EVAL_FALL_TILT_DEG", 50.0)
        self.fall_z = _envf("EVAL_FALL_Z_M", 0.40)     # base this low == fallen
        self.tilt_free = _envf("EVAL_TILT_FREE_DEG", 12.0)  # no tilt penalty up to this
        self.tilt_w = _envf("EVAL_TILT_W", 150.0)      # max tilt deduction
        self.squat_w = _envf("EVAL_SQUAT_W", 150.0)    # max squat deduction
        self.joint_w = _envf("EVAL_JOINT_W", 150.0)    # max joint-range deduction
        # Fraction of each joint's range treated as still-normal PAST its soft limit
        # before penalizing. 0.0 == penalize only joints genuinely outside the soft
        # limits (a normal stand keeps knees/ankles/hips near — but inside — their
        # limits, so it must score ~100). Raise to tighten the "normal" band.
        self.joint_margin = _envf("EVAL_JOINT_MARGIN", 0.0)
        self.joint_cap = _envf("EVAL_JOINT_CAP", 1.0)  # summed out-of-band frac that == full joint_w
        self.pub_hz = _envf("EVAL_PUB_HZ", 10.0)       # publish cadence in sim-time

        # ---- state ----
        self._armed = False
        self._done = False
        self._t0 = 0.0
        self._p0 = None          # (x, y) start location
        self._z0 = None          # standing height at arm-time (squat reference)
        self._last_pub_t = -1e18
        self._final = None       # latched terminal payload
        self._err_once = False

        try:
            self.pub = ChannelPublisher(self.topic, String_)
            self.pub.Init()
            print(
                f"[stand_eval:{self.env_id}] publishing {self.topic} (String JSON)",
                flush=True,
            )
        except Exception as e:  # noqa: BLE001 - never break bring-up
            self.pub = None
            print(f"[stand_eval] publisher init failed: {e}", flush=True)

    # -- lifecycle -----------------------------------------------------------
    def arm(self):
        """Begin the timed stand: capture start location + standing height."""
        pos, quat, _, _ = self._read_root()
        if pos is None:
            print("[stand_eval] arm failed: no root state", flush=True)
            return
        self._p0 = (pos[0], pos[1])
        self._z0 = pos[2]
        self._t0 = self.sim_time()
        self._armed = True
        self._done = False
        self._final = None
        self._last_pub_t = -1e18
        print(f"[stand_eval:{self.env_id}] ARMED at t={self._t0:.2f}s p0=({pos[0]:.2f},{pos[1]:.2f}) "
              f"z0={pos[2]:.3f} window={self.window_s:.0f}s", flush=True)

    def disarm(self):
        """Return to pre-eval hold (start-hold: term 0, result 100)."""
        self._armed = False
        self._done = False
        self._final = None
        print(f"[stand_eval:{self.env_id}] DISARMED (start-hold)", flush=True)

    # -- per-step ------------------------------------------------------------
    def update(self):
        """Compute + publish (sim-time throttled). Safe to call every loop."""
        try:
            self._update()
        except Exception as e:  # noqa: BLE001 - the eval must never crash the sim
            if not self._err_once:
                self._err_once = True
                print(f"[stand_eval:{self.env_id}] update error (silenced): {e}", flush=True)

    def _update(self):
        if self.pub is None:
            return
        t = self.sim_time()

        # Latched terminal state: keep re-publishing the final verdict.
        if self._done and self._final is not None:
            self._maybe_pub(t, self._final)
            return

        # Not yet balancing: holding upright at the start == perfect start score.
        if not self._armed:
            pos, quat, _, _ = self._read_root()
            self._maybe_pub(t, {
                "t": 0.0, "termination": 0.0, "result": 100.0,
                "fallen": False, "standing": False,
                "dist": 0.0, "height": 0.0, "tilt": 0.0, "reason": "start-hold",
                **self._pose_fields(pos, quat),
            })
            return

        elapsed = max(0.0, t - self._t0)
        pos, quat, finite_root, finite_joint = self._read_root()

        # ---- fall detection (hard failure) --------------------------------
        fall_reason = None
        if pos is None or not finite_root or not finite_joint:
            fall_reason = "fall:nonfinite"
        else:
            tilt = self._tilt_deg(quat)
            if tilt > self.fall_tilt:
                fall_reason = "fall:tilt"
            elif pos[2] < self.fall_z:
                fall_reason = "fall:low"
        if fall_reason is not None:
            payload = {
                "t": round(elapsed, 3), "termination": 1.0, "result": -100.0,
                "fallen": True, "standing": False,
                "dist": round(self._dist(pos), 3) if pos else -1.0,
                "height": round(pos[2], 3) if pos else -1.0,
                "tilt": round(self._tilt_deg(quat), 2) if pos else -1.0,
                "reason": fall_reason,
                **self._pose_fields(pos, quat),
            }
            self._done = True
            self._final = payload
            print(f"[stand_eval:{self.env_id}] FALL ({fall_reason}) at t={elapsed:.2f}s -> result=-100", flush=True)
            self._maybe_pub(t, payload)
            return

        # ---- standing: score how good the stand is ------------------------
        tilt = self._tilt_deg(quat)
        dist = self._dist(pos)
        dist_pen = min(dist / self.dist_max, 1.0) * 199.0

        # tilt: leaning off vertical (any direction). Free up to tilt_free deg,
        # then ramps to full weight at the fall tilt.
        tilt_span = max(self.fall_tilt - self.tilt_free, 1e-3)
        tilt_frac = min(max((tilt - self.tilt_free) / tilt_span, 0.0), 1.0)
        tilt_pen = tilt_frac * self.tilt_w

        # squat: base dropping from its standing height toward the fall height.
        span = max(self._z0 - self.fall_z, 1e-3)
        squat_frac = min(max((self._z0 - pos[2]) / span, 0.0), 1.0)
        squat_pen = squat_frac * self.squat_w

        joint_pen = self._joint_penalty()

        result = 100.0 - dist_pen - tilt_pen - squat_pen - joint_pen
        result = max(-99.0, min(100.0, result))

        termination = min(elapsed / self.window_s, 1.0)
        success = termination >= 1.0
        payload = {
            "t": round(elapsed, 3), "termination": round(termination, 4),
            "result": round(result, 2), "fallen": False, "standing": True,
            "dist": round(dist, 3), "height": round(pos[2], 3),
            "tilt": round(tilt, 2), "reason": "success" if success else "standing",
            **self._pose_fields(pos, quat),
        }
        if success:
            self._done = True
            self._final = payload
            print(f"[stand_eval:{self.env_id}] SUCCESS: {self.window_s:.0f}s stand complete -> "
                  f"result={result:.1f}", flush=True)
        self._maybe_pub(t, payload)

    # -- helpers -------------------------------------------------------------
    def _maybe_pub(self, t, payload):
        if t - self._last_pub_t < 1.0 / max(self.pub_hz, 1e-6):
            return
        self._last_pub_t = t
        m = std_msgs_msg_dds__String_()
        payload = dict(payload)
        payload["robot_id"] = self.env_id
        m.data = json.dumps(payload, separators=(",", ":"))
        self.pub.Write(m)

    def _read_root(self):
        """(pos[x,y,z], quat[w,x,y,z], root_finite, joint_finite) or Nones."""
        import torch
        data = self.robot.data
        rs = data.root_state_w[self.env_id]
        finite_root = bool(torch.isfinite(rs).all())
        finite_joint = bool(torch.isfinite(data.joint_pos[self.env_id]).all())
        rs = rs.detach().float().cpu().tolist()
        pos = rs[0:3]
        quat = rs[3:7]
        return pos, quat, finite_root, finite_joint

    def _joint_penalty(self):
        """Summed out-of-band fraction across joints, scaled to [0, joint_w]."""
        import torch
        data = self.robot.data
        lim = getattr(data, "soft_joint_pos_limits", None)
        if lim is None:
            lim = getattr(data, "joint_pos_limits", None)
        if lim is None:
            return 0.0
        q = data.joint_pos[self.env_id].detach().float().cpu().tolist()
        lim = lim[self.env_id].detach().float().cpu().tolist()  # [J, 2]
        total = 0.0
        for qi, (lo, hi) in zip(q, lim):
            rng = hi - lo
            if rng <= 1e-6:
                continue
            m = self.joint_margin * rng
            lo_ok, hi_ok = lo + m, hi - m
            excess = max(lo_ok - qi, 0.0) + max(qi - hi_ok, 0.0)
            total += excess / rng
        frac = min(total / max(self.joint_cap, 1e-6), 1.0)
        return frac * self.joint_w

    def _dist(self, pos):
        if pos is None or self._p0 is None:
            return 0.0
        dx = pos[0] - self._p0[0]
        dy = pos[1] - self._p0[1]
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def _pose_fields(pos, quat):
        """Absolute root pose for synchronized external telemetry consumers."""
        if pos is None or quat is None:
            return {"position": None, "quaternion": None}
        return {
            "position": [round(float(v), 6) for v in pos],
            "quaternion": [round(float(v), 7) for v in quat],
        }

    @staticmethod
    def _tilt_deg(quat):
        """Base tilt from vertical (deg) given an (w, x, y, z) quaternion."""
        _, x, y, _ = quat
        c = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
        return math.degrees(math.acos(c))
