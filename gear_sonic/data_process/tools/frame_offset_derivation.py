#!/usr/bin/env python3
"""Zero-pose forward-kinematics derivation of VR-3-point / reward-point body
offsets for the TienKung 2dex robot, in the same SEMANTIC geometry as the G1
baseline they were inherited from.

Pure-numpy URDF parser + FK. No Isaac / mujoco dependency. Run on the Mac.

Context
-------
gear_sonic .../mdp/commands.py applies each offset in the tracked body's LOCAL
frame:  world_point = body_pos_w + quat_apply(body_quat_w, offset).
So `offset` is a vector expressed in the body link frame. The G1 numbers
(hand: [0.18, -/+0.025, 0]; torso->HMD: [0,0,0.35]; H2 chest: [0,0,0.5]) were
authored against G1/H2 link frames whose axis conventions differ from TienKung.
This script re-expresses the same physical points in TienKung link frames.

Usage: python frame_offset_derivation.py <g1.urdf> <tiangong2dex.urdf>
"""
import sys
import numpy as np
import xml.etree.ElementTree as ET

np.set_printoptions(precision=4, suppress=True)


def rpy_to_matrix(r, p, y):
    """URDF fixed-axis roll-pitch-yaw (R = Rz(y) @ Ry(p) @ Rx(r))."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def parse_urdf(path):
    """Return dict: joints[name] = {parent, child, xyz, rpy, axis, type}."""
    tree = ET.parse(path)
    root = tree.getroot()
    joints = {}
    child_to_joint = {}
    links = set()
    for link in root.findall("link"):
        links.add(link.get("name"))
    for j in root.findall("joint"):
        name = j.get("name")
        jtype = j.get("type")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        origin = j.find("origin")
        if origin is not None:
            xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
            rpy = np.array([float(v) for v in origin.get("rpy", "0 0 0").split()])
        else:
            xyz = np.zeros(3)
            rpy = np.zeros(3)
        joints[name] = dict(parent=parent, child=child, xyz=xyz, rpy=rpy, type=jtype)
        child_to_joint[child] = name
    return joints, child_to_joint, links


def fk_link(link, joints, child_to_joint):
    """Zero-pose world transform (R 3x3, t 3) of a link frame relative to root.

    At zero joint angle every joint contributes only its <origin> transform,
    so world pose = product of origins from root down to the link.
    """
    chain = []
    cur = link
    while cur in child_to_joint:
        jname = child_to_joint[cur]
        chain.append(jname)
        cur = joints[jname]["parent"]
    chain.reverse()
    R = np.eye(3)
    t = np.zeros(3)
    for jname in chain:
        j = joints[jname]
        Rj = rpy_to_matrix(*j["rpy"])
        t = t + R @ j["xyz"]
        R = R @ Rj
    return R, t


def report_link(name, joints, c2j):
    R, t = fk_link(name, joints, c2j)
    print(f"  {name}")
    print(f"    world pos : {t}")
    print(f"    world R (cols = local x,y,z in world):\n{np.array2string(R, prefix='      ')}")
    return R, t


def local_from_world_dir(R, world_dir):
    """Express a world-frame direction in the link's local frame."""
    return R.T @ world_dir


def main():
    # Default to the vendored URDFs relative to the repo root (this file lives at
    # gear_sonic/data_process/tools/); override by passing <g1.urdf> <tk.urdf>.
    here = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
    repo = __import__("os").path.abspath(__import__("os").path.join(here, "..", "..", ".."))
    base = __import__("os").path.join(repo, "gear_sonic", "data", "assets", "robot_description", "urdf")
    default_g1 = __import__("os").path.join(base, "g1", "main.urdf")
    default_tk = __import__("os").path.join(base, "tiangong2dex", "tiangong2dex.urdf")
    if len(sys.argv) >= 3:
        g1_path, tk_path = sys.argv[1], sys.argv[2]
    else:
        g1_path, tk_path = default_g1, default_tk

    print("=" * 78)
    print("G1 baseline (source of inherited offsets)")
    print("=" * 78)
    gj, gc2j, glinks = parse_urdf(g1_path)
    print("[links present]",
          "torso_link" in glinks, "left_wrist_yaw_link" in glinks,
          "right_wrist_yaw_link" in glinks, "head_link" in glinks)
    print("\n-- G1 link frames at zero pose --")
    Rg_lw, tg_lw = report_link("left_wrist_yaw_link", gj, gc2j)
    Rg_rw, tg_rw = report_link("right_wrist_yaw_link", gj, gc2j)
    Rg_torso, tg_torso = report_link("torso_link", gj, gc2j)
    # supporting links for direction semantics
    Rg_le, tg_le = fk_link("left_elbow_link", gj, gc2j)
    Rg_re, tg_re = fk_link("right_elbow_link", gj, gc2j)
    Rg_head, tg_head = fk_link("head_link", gj, gc2j)
    Rg_pelvis, tg_pelvis = fk_link("pelvis", gj, gc2j)
    print(f"\n  left_elbow_link  world pos: {tg_le}")
    print(f"  right_elbow_link world pos: {tg_re}")
    print(f"  head_link        world pos: {tg_head}")
    print(f"  pelvis           world pos: {tg_pelvis}")

    # G1 offsets (inherited)
    g1_lh = np.array([0.18, -0.025, 0.0])
    g1_rh = np.array([0.18, +0.025, 0.0])
    g1_torso_hmd = np.array([0.0, 0.0, 0.35])
    # world points G1 offsets produce
    print("\n-- G1 inherited offsets -> world points (zero pose) --")
    p_lh = tg_lw + Rg_lw @ g1_lh
    p_rh = tg_rw + Rg_rw @ g1_rh
    p_hmd = tg_torso + Rg_torso @ g1_torso_hmd
    print(f"  left  hand pt : {p_lh}   (world dir of offset: {Rg_lw @ g1_lh})")
    print(f"  right hand pt : {p_rh}   (world dir of offset: {Rg_rw @ g1_rh})")
    print(f"  HMD  pt (torso+0.35z_local): {p_hmd}   world dir: {Rg_torso @ g1_torso_hmd}")
    # G1 hand: which world direction does +x_local (0.18) point?
    print(f"\n  G1 left  wrist local +x in world: {Rg_lw @ [1,0,0]}")
    print(f"  G1 left  wrist local +y in world: {Rg_lw @ [0,1,0]}")
    print(f"  G1 left  wrist->elbow world vec  : {tg_le - tg_lw}")
    print(f"  G1 torso local +z in world       : {Rg_torso @ [0,0,1]}")
    print(f"  G1 head_z - torso_z              : {tg_head[2] - tg_torso[2]:.4f}")
    print(f"  G1 head_z - pelvis_z (stature-ish): {tg_head[2] - tg_pelvis[2]:.4f}")
    # G1 hand geometry: how far is wrist_yaw from palm / fingertip? Does 0.18 fit?
    Rg_palm, tg_palm = fk_link("left_hand_palm_link", gj, gc2j)
    Rg_ftip, tg_ftip = fk_link("left_hand_middle_1_link", gj, gc2j)
    Rg_sh, tg_sh = fk_link("left_shoulder_pitch_link", gj, gc2j)
    print(f"\n  G1 left_hand_palm     world pos: {tg_palm}  (|wrist->palm|={np.linalg.norm(tg_palm-tg_lw):.4f})")
    print(f"  G1 left_hand_middletip world pos: {tg_ftip}  (|wrist->tip| ={np.linalg.norm(tg_ftip-tg_lw):.4f})")
    print(f"  G1 0.18-offset landed at        : {p_lh}  (vs palm {tg_palm}, tip {tg_ftip})")
    print(f"  G1 left_shoulder_pitch world pos: {tg_sh}  (z above pelvis={tg_sh[2]-tg_pelvis[2]:.4f})")
    print(f"  G1 HMD z above pelvis           : {p_hmd[2]-tg_pelvis[2]:.4f}  (HMD above shoulder={p_hmd[2]-tg_sh[2]:.4f})")

    # ---- H2 (source of the reward_point [0,0,0.5] chest/upper-body offset) ----
    h2_path = g1_path.replace("/g1/main.urdf", "/h2/h2.urdf")
    print("\n" + "=" * 78)
    print("H2 baseline (source of reward_point [0,0,0.5])")
    print("=" * 78)
    hj, hc2j, hlinks = parse_urdf(h2_path)
    Rh_torso, th_torso = fk_link("torso_link", hj, hc2j)
    Rh_hp, th_hp = fk_link("head_pitch_link", hj, hc2j)
    Rh_hy, th_hy = fk_link("head_yaw_link", hj, hc2j)
    Rh_sh, th_sh = fk_link("left_shoulder_pitch_link", hj, hc2j)
    Rh_pel, th_pel = fk_link("pelvis", hj, hc2j)
    h2_reward_pt = th_torso + Rh_torso @ np.array([0, 0, 0.5])
    print(f"  torso_link       world pos: {th_torso}")
    print(f"  head_pitch_link  world pos: {th_hp}  (above torso {th_hp[2]-th_torso[2]:.3f})")
    print(f"  head_yaw_link    world pos: {th_hy}  (above torso {th_hy[2]-th_torso[2]:.3f})")
    print(f"  shoulder_pitch_l world pos: {th_sh}  (above torso {th_sh[2]-th_torso[2]:.3f})")
    print(f"  reward pt torso+0.5z_local: {h2_reward_pt}")
    print(f"    -> above pelvis {h2_reward_pt[2]-th_pel[2]:.3f}; above shoulder {h2_reward_pt[2]-th_sh[2]:.3f};"
          f" vs head_yaw {h2_reward_pt[2]-th_hy[2]:+.3f}")
    print(f"    torso above pelvis: {th_torso[2]-th_pel[2]:.3f}")
    print(f"    frac torso->head_yaw covered by 0.5: {0.5/(th_hy[2]-th_torso[2]):.3f}")

    print("\n" + "=" * 78)
    print("TienKung 2dex")
    print("=" * 78)
    tj, tc2j, tlinks = parse_urdf(tk_path)
    print("\n-- TienKung link frames at zero pose --")
    Rt_lw, tt_lw = report_link("wrist_roll_l_link", tj, tc2j)
    Rt_rw, tt_rw = report_link("wrist_roll_r_link", tj, tc2j)
    Rt_waist, tt_waist = report_link("waist_pitch_link", tj, tc2j)

    # supporting links
    Rt_le, tt_le = fk_link("elbow_pitch_l_link", tj, tc2j)
    Rt_re, tt_re = fk_link("elbow_pitch_r_link", tj, tc2j)
    Rt_ltcp, tt_ltcp = fk_link("left_tcp_link", tj, tc2j)
    Rt_rtcp, tt_rtcp = fk_link("right_tcp_link", tj, tc2j)
    Rt_headp, tt_headp = fk_link("head_pitch_link", tj, tc2j)
    Rt_camhead, tt_camhead = fk_link("camera_head_link", tj, tc2j)
    Rt_pelvis, tt_pelvis = fk_link("pelvis", tj, tc2j)
    print(f"\n  elbow_pitch_l_link world pos: {tt_le}")
    print(f"  elbow_pitch_r_link world pos: {tt_re}")
    print(f"  left_tcp_link       world pos: {tt_ltcp}")
    print(f"  right_tcp_link      world pos: {tt_rtcp}")
    print(f"  head_pitch_link     world pos: {tt_headp}")
    print(f"  camera_head_link    world pos: {tt_camhead}")
    print(f"  pelvis              world pos: {tt_pelvis}")

    # Direction semantics for TienKung wrist frames
    print("\n-- TienKung wrist local axes in world --")
    for axname, ax in [("+x", [1, 0, 0]), ("+y", [0, 1, 0]), ("+z", [0, 0, 1])]:
        print(f"  L wrist local {axname} in world: {Rt_lw @ ax}")
    print(f"  L wrist->tcp world vec          : {tt_ltcp - tt_lw}")
    print(f"  L elbow->wrist world vec        : {tt_lw - tt_le}")
    for axname, ax in [("+x", [1, 0, 0]), ("+y", [0, 1, 0]), ("+z", [0, 0, 1])]:
        print(f"  R wrist local {axname} in world: {Rt_rw @ ax}")
    print(f"  R wrist->tcp world vec          : {tt_rtcp - tt_rw}")

    print("\n-- TienKung waist_pitch local axes in world --")
    for axname, ax in [("+x", [1, 0, 0]), ("+y", [0, 1, 0]), ("+z", [0, 0, 1])]:
        print(f"  waist local {axname} in world: {Rt_waist @ ax}")
    print(f"  waist_pitch world z             : {tt_waist[2]:.4f}")
    print(f"  head_pitch_z - waist_pitch_z    : {tt_headp[2]-tt_waist[2]:.4f}")
    print(f"  camera_head_z - waist_pitch_z   : {tt_camhead[2]-tt_waist[2]:.4f}")
    print(f"  head_pitch_z - pelvis_z (stature): {tt_headp[2]-tt_pelvis[2]:.4f}")

    # ---- Build TienKung offsets reproducing G1 semantics ----
    print("\n" + "=" * 78)
    print("DERIVATION of TienKung offsets")
    print("=" * 78)

    # HAND: 0.18 m along wrist->hand (tcp) direction, expressed in wrist local
    # frame; plus 0.025 m lateral toward body midline. We decompose using the
    # tcp direction for the 'toward hand' axis, and the world-Y (robot lateral)
    # for the midline axis, both re-expressed in the wrist local frame.
    def hand_offset(R_wrist, t_wrist, t_tcp, side, mag_fwd=0.18, mag_lat=0.025):
        fwd_world = (t_tcp - t_wrist)
        fwd_world = fwd_world / np.linalg.norm(fwd_world)
        # midline direction in world: toward y=0 plane. Left arm sits at +Y so
        # toward midline = -Y; right arm at -Y so toward midline = +Y.
        if side == "L":
            mid_world = np.array([0.0, -1.0, 0.0])
        else:
            mid_world = np.array([0.0, +1.0, 0.0])
        off_world = mag_fwd * fwd_world + mag_lat * mid_world
        off_local = R_wrist.T @ off_world
        return off_local, off_world, fwd_world

    lh_local, lh_world, lfwd = hand_offset(Rt_lw, tt_lw, tt_ltcp, "L")
    rh_local, rh_world, rfwd = hand_offset(Rt_rw, tt_rw, tt_rtcp, "R")
    print("\n[HAND] TienKung wrist-local offsets (0.18 fwd + 0.025 midline):")
    print(f"  L wrist->tcp unit dir (world): {lfwd}")
    print(f"  L offset world : {lh_world}")
    print(f"  L offset LOCAL : {lh_local}")
    print(f"  R wrist->tcp unit dir (world): {rfwd}")
    print(f"  R offset world : {rh_world}")
    print(f"  R offset LOCAL : {rh_local}")

    # Sanity: resulting world points
    p_lh_tk = tt_lw + Rt_lw @ lh_local
    p_rh_tk = tt_rw + Rt_rw @ rh_local
    print(f"  -> L hand world pt: {p_lh_tk}")
    print(f"  -> R hand world pt: {p_rh_tk}")
    print(f"     (tcp world pos L/R for reference: {tt_ltcp} / {tt_rtcp})")

    # HEAD (vr_3point torso row): put point at head/HMD height above waist_pitch.
    # G1 torso->HMD local offset is +0.35 z_local == +0.35 world-up (torso z is
    # world-up at zero pose). Use the ACTUAL geometric rise waist_pitch->head as
    # the primary value, cross-checked against a height-ratio scaling of 0.35.
    up_local_waist = Rt_waist.T @ np.array([0.0, 0.0, 1.0])  # world-up in waist local
    head_rise = tt_camhead[2] - tt_waist[2]   # waist -> HMD/camera height
    # G1 head_link is degenerate (co-located w/ pelvis); use pelvis-spawn heights
    # for the stature ratio instead (G1 ~0.74 m, TienKung 0.97 m).
    G1_SPAWN_Z, TK_SPAWN_Z = 0.74, 0.97
    ratio = TK_SPAWN_Z / G1_SPAWN_Z
    scaled_035 = 0.35 * ratio
    print("\n[HEAD] waist_pitch-local up axis (world-up in local):", up_local_waist)
    print(f"  actual waist->camera_head rise : {head_rise:.4f} m")
    print(f"  height ratio TK/G1 (spawn z)   : {ratio:.4f}")
    print(f"  G1 0.35 * ratio                : {scaled_035:.4f} m (cross-check vs geometry)")
    hmd_mag = head_rise
    hmd_local = up_local_waist * hmd_mag
    print(f"  chosen HMD rise                 : {hmd_mag:.4f} m")
    print(f"  HEAD offset LOCAL (waist frame) : {hmd_local}")
    p_hmd_tk = tt_waist + Rt_waist @ hmd_local
    print(f"  -> HMD world pt: {p_hmd_tk}  (camera_head actual: {tt_camhead})")

    # CHEST/UPPER-BODY (reward_point waist row): H2 used [0,0,0.5] above its
    # torso_link. On H2 that point lands near the HEAD (H2 head_yaw is
    # torso+0.554; 0.5 is ~0.90 of the torso->head_yaw rise, i.e. head/neck
    # region), NOT the chest. Reproduce the SAME anatomical target on TienKung:
    # apply the same fraction of TienKung's waist_pitch->head_yaw rise.
    Rt_sh_l, tt_sh_l = fk_link("shoulder_pitch_l_link", tj, tc2j)
    Rt_headyaw, tt_headyaw = fk_link("head_yaw_link", tj, tc2j)
    h2_torso_to_headyaw = th_hy[2] - th_torso[2]
    h2_frac = 0.5 / h2_torso_to_headyaw           # fraction of torso->head_yaw
    tk_waist_to_headyaw = tt_headyaw[2] - tt_waist[2]
    reward_rise = h2_frac * tk_waist_to_headyaw   # same fraction on TienKung
    reward_local = up_local_waist * reward_rise
    p_reward = tt_waist + Rt_waist @ reward_local
    print("\n[CHEST/UPPER-BODY] reward_point waist row (H2 [0,0,0.5]):")
    print(f"  H2 torso->head_yaw rise        : {h2_torso_to_headyaw:.4f} m; 0.5 covers frac {h2_frac:.3f}")
    print(f"  TK waist->head_yaw rise        : {tk_waist_to_headyaw:.4f} m")
    print(f"  TK shoulder_pitch_l world pos  : {tt_sh_l}")
    print(f"  TK head_yaw world pos          : {tt_headyaw}")
    print(f"  reward rise = frac * TK rise   : {reward_rise:.4f} m")
    print(f"  cross-check 0.5*spawn_ratio    : {0.5*ratio:.4f} m")
    print(f"  REWARD offset LOCAL (waist)    : {reward_local}")
    print(f"  -> reward world pt: {p_reward}  (waist {tt_waist[2]:.3f}, shoulder {tt_sh_l[2]:.3f}, head_yaw {tt_headyaw[2]:.3f})")
    chest_local = reward_local

    # ---- Final rounded proposals ----
    print("\n" + "=" * 78)
    print("PROPOSED TienKung offsets (rounded to 3 dp)")
    print("=" * 78)
    r3 = lambda v: [round(float(x), 3) for x in v]
    print("vr_3point_body order [wrist_roll_l, wrist_roll_r, waist_pitch]:")
    print("  L hand :", r3(lh_local))
    print("  R hand :", r3(rh_local))
    print("  HEAD   :", r3(hmd_local))
    print("reward_point_body order [waist_pitch, wrist_roll_l, wrist_roll_r]:")
    print("  CHEST  :", r3(chest_local))
    print("  L wrist: [0.0, 0.0, 0.0]  (kept zero, as inherited)")
    print("  R wrist: [0.0, 0.0, 0.0]  (kept zero, as inherited)")


if __name__ == "__main__":
    main()
