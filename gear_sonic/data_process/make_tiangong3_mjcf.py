#!/usr/bin/env python3  # noqa: EXE001
# ruff: noqa: T201, DOC
"""Generate the vendored TienKung 3 (tiangong3) MJCF from its URDF.

25-DOF sibling of the 2dex asset pipeline. Unlike tiangong2dex -- which was
VENDORED from an upstream ``*_torq.xml`` and then range-tightened with
``tighten_mjcf_ranges_to_urdf_tiangong2dex.py`` -- tiangong3 has NO published
MJCF, so this script COMPILES one from ``tiangong3.urdf`` with MuJoCo. Because
it is compiled directly from the authoritative URDF, the hinge ``range`` values
already equal the URDF ``<limit lower/upper>`` and NO separate tightening pass
is needed (this script asserts that equality at the end).

Design decisions (documented so the asset is reproducible and auditable):

  * MESH-FREE. The retargeter loads this MJCF via ``newton.ModelBuilder().add_mjcf()``
    for FK/IK only (collision_weight=0 in the retargeter config), so geoms/meshes
    are unnecessary. We strip every URDF ``<visual>``/``<collision>`` BEFORE the
    MuJoCo compile -- which also sidesteps the fact that several tiangong3 STL
    meshes (e.g. pelvis.STL) exceed MuJoCo's 200k-face decoder limit and cannot
    be compiled at all. Bodies keep their full mass/inertia via ``<inertial>``.
    This is the "mesh-free MJCF" fallback documented in the retarget README.

  * FLOATING BASE. A plain URDF compile welds the root ``pelvis`` link to the
    world (fixed base). The retargeter needs a free-floating pelvis so the root
    can translate/rotate with the motion, exactly like the 2dex MJCF
    (``<body name="pelvis"><freejoint name="floating_base"/> ...``). We therefore
    re-wrap the three worldbody root children (hip_pitch_l_link, hip_pitch_r_link,
    waist_yaw_link) under a synthesized ``pelvis`` body carrying a ``<freejoint>``
    and the pelvis ``<inertial>`` read from the URDF. Pelvis spawn height
    (0.94236470 m) is copied from the 2dex MJCF -- the legs are geometrically
    identical (ankle_roll_l at pelvis-frame z = -0.9299 in both), so the rest
    stance height matches.

Result: worldbody = world -> pelvis(freejoint) -> {legs, waist->head, waist->arms};
26 bodies (pelvis + 25 DOF links), 25 hinge joints + 1 freejoint. Joint document
order == convert_soma_csv_to_motion_lib_tiangong3.MUJOCO_JOINT_NAMES.

    python3 gear_sonic/data_process/make_tiangong3_mjcf.py            # write the MJCF
    python3 gear_sonic/data_process/make_tiangong3_mjcf.py --verify   # recompile + assert, no write
"""

import argparse
import os
import xml.etree.ElementTree as ET

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.normpath(
    os.path.join(HERE, "..", "data/assets/robot_description/urdf/tiangong3/tiangong3.urdf")
)
MJCF_OUT = os.path.normpath(
    os.path.join(HERE, "..", "data/assets/robot_description/mjcf/tiangong3.xml")
)

# Pelvis spawn height, copied from the 2dex MJCF (identical leg geometry).
PELVIS_REST_Z = 0.94236470

# NOTE: XML comments may not contain the '--' token, so this text avoids it.
HEADER = (
    "Generated from tiangong3.urdf by gear_sonic/data_process/make_tiangong3_mjcf.py.\n"
    "     MESH-FREE (visual/collision stripped before compile; bodies keep mass via\n"
    "     inertial). Used by the SOMA retargeter newton.add_mjcf for FK/IK only\n"
    "     (retargeter collision_weight=0). Floating base: the URDF root 'pelvis' is\n"
    "     re-wrapped as a free body (freejoint floating_base) with the three root\n"
    "     children (hip_pitch_l/r, waist_yaw) as its subtree, matching the 2dex MJCF\n"
    "     layout. Hinge ranges equal the URDF limit (compiled from URDF; no separate\n"
    "     tightening pass needed). 26 bodies (pelvis + 25 DOF), 25 hinges."
)


def strip_meshes(urdf_path):
    """Parse the URDF and remove every <visual>/<collision> (mesh-free)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                link.remove(e)
    return tree, root


def pelvis_inertial(urdf_root):
    """Return the URDF pelvis <inertial> as (pos_xyz_str, mass_str, fullinertia_str)."""
    for link in urdf_root.findall("link"):
        if link.get("name") == "pelvis":
            ine = link.find("inertial")
            o = ine.find("origin")
            m = ine.find("mass")
            ia = ine.find("inertia")
            pos = o.get("xyz")
            mass = m.get("value")
            # MuJoCo fullinertia order: ixx iyy izz ixy ixz iyz
            full = " ".join(
                ia.get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            )
            return pos, mass, full
    raise RuntimeError("no pelvis link in URDF")


def build(urdf_path):
    """Compile a mesh-free MJCF from the URDF and re-wrap a floating pelvis. Returns XML text."""
    tree, urdf_root = strip_meshes(urdf_path)
    pel_pos, pel_mass, pel_full = pelvis_inertial(urdf_root)

    # Compile the mesh-free URDF with MuJoCo (root pelvis welded to world).
    import tempfile

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as fh:
        tree.write(fh.name)
        tmp_urdf = fh.name
    m = mujoco.MjModel.from_xml_path(tmp_urdf)
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as fh:
        raw_path = fh.name
    mujoco.mj_saveLastXML(raw_path, m)
    os.unlink(tmp_urdf)

    mtree = ET.parse(raw_path)
    mroot = mtree.getroot()
    os.unlink(raw_path)
    mroot.set("model", "tiangong3")

    wb = mroot.find("worldbody")
    root_children = list(wb.findall("body"))  # hip_pitch_l, hip_pitch_r, waist_yaw

    # Synthesize the floating pelvis body and move the root children under it.
    pelvis = ET.Element(
        "body",
        {"name": "pelvis", "pos": f"0 0 {PELVIS_REST_Z}", "quat": "1 0 0 0"},
    )
    ET.SubElement(pelvis, "freejoint", {"name": "floating_base"})
    ET.SubElement(
        pelvis,
        "inertial",
        {"pos": pel_pos, "mass": pel_mass, "fullinertia": pel_full},
    )
    for b in root_children:
        wb.remove(b)
        pelvis.append(b)
    wb.append(pelvis)

    # Insert the provenance comment right after the <mujoco ...> open tag.
    ET.indent(mtree, space="  ")
    body = ET.tostring(mroot, encoding="unicode")
    return f"<!-- {HEADER} -->\n{body}"


def validate(xml_text):
    """Recompile the produced MJCF and assert structure + ranges == URDF."""
    m = mujoco.MjModel.from_xml_string(xml_text)
    bn = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)]
    jn = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    hinge = [
        jn[i] for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE
    ]
    assert bn[0] == "world" and bn[1] == "pelvis", f"bad body head {bn[:2]}"
    assert len(bn) == 27, f"expected 27 bodies (world+pelvis+25), got {len(bn)}"
    assert len(hinge) == 25, f"expected 25 hinges, got {len(hinge)}"
    free = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE]
    assert len(free) == 1, f"expected exactly 1 free joint, got {len(free)}"

    # ranges == URDF <limit>
    ur = ET.parse(URDF).getroot()
    ulim = {}
    for j in ur.findall("joint"):
        if j.get("type") == "fixed":
            continue
        lim = j.find("limit")
        if lim is not None:
            ulim[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    for i in range(m.njnt):
        if m.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = jn[i]
        lo, hi = m.jnt_range[i]
        ulo, uhi = ulim[name]
        assert abs(lo - ulo) < 1e-4 and abs(hi - uhi) < 1e-4, (
            f"range mismatch {name}: mjcf=({lo},{hi}) urdf=({ulo},{uhi})"
        )
    print(f"[validate] OK: {len(bn)} bodies, {len(hinge)} hinges, 1 freejoint, ranges==URDF")
    print(f"[validate] elbow_pitch_l_joint hinge index = {hinge.index('elbow_pitch_l_joint')}")
    return hinge


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--out", default=MJCF_OUT)
    ap.add_argument("--verify", action="store_true", help="recompile existing MJCF + assert, no write")
    args = ap.parse_args()

    if args.verify:
        validate(open(args.out).read())
        return

    xml_text = build(args.urdf)
    validate(xml_text)
    with open(args.out, "w") as f:
        f.write(xml_text)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
