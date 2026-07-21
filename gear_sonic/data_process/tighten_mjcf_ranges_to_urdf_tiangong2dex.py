"""Tighten the vendored TienKung 2dex MJCF joint ``range=`` attrs to the URDF limits.

The vendored MJCF ``tiangong2dex.xml`` inherited joint ``range`` values from the
CAD export that are ~0.07-0.12 rad WIDER than the authoritative URDF
``<limit lower/upper>`` on a set of hinge joints (shoulders / elbows / wrists,
hip_pitch, hip_yaw, waist_yaw, head_pitch). The SOMA retargeter parses this MJCF
via ``newton.ModelBuilder().add_mjcf()`` and clamps retargeted DOFs to the joint
ranges, so DOFs could exceed the real URDF limits. The URDF is authoritative;
this script rewrites every hinge joint's ``range`` to exactly the URDF
``lower upper`` so the clamper enforces the real limits on the next retarget.

Surgical by design: it does NOT re-serialize the XML (which would reflow the
whole file). It parses both files with ``xml.etree`` to build the authoritative
name -> (lower, upper) map, then does a targeted text substitution of just the
``range="..."`` attribute on each joint element, leaving all other bytes intact.
The freejoint, bodies, actuators, defaults, and assets are untouched.

    python3 gear_sonic/data_process/tighten_mjcf_ranges_to_urdf_tiangong2dex.py            # apply in place
    python3 gear_sonic/data_process/tighten_mjcf_ranges_to_urdf_tiangong2dex.py --dry-run  # report only
    python3 gear_sonic/data_process/tighten_mjcf_ranges_to_urdf_tiangong2dex.py --verify   # re-parse + assert
"""

import argparse
import os
import re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(__file__)
MJCF = os.path.normpath(
    os.path.join(HERE, "..", "data/assets/robot_description/mjcf/tiangong2dex.xml")
)
URDF = os.path.normpath(
    os.path.join(
        HERE, "..", "data/assets/robot_description/urdf/tiangong2dex/tiangong2dex.urdf"
    )
)


def urdf_limits(path):
    """name -> (lower, upper) for every non-fixed URDF joint carrying a <limit>."""
    root = ET.parse(path).getroot()
    out = {}
    for j in root.findall("joint"):
        if j.get("type") == "fixed":
            continue
        lim = j.find("limit")
        if lim is None:
            continue
        out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


def mjcf_hinge_ranges(path):
    """name -> (lower, upper) for every hinge joint that carries a range attr."""
    root = ET.parse(path).getroot()
    out = {}
    for j in root.iter("joint"):
        if j.get("type") == "hinge" and j.get("range") is not None:
            lo, hi = j.get("range").split()
            out[j.get("name")] = (float(lo), float(hi))
    return out


def rewrite(text, name, lower, upper):
    """Replace the range attr of the joint element named ``name`` in raw MJCF text."""
    new_range = f"{lower!r} {upper!r}"
    # Match a single <joint ... name="NAME" ... range="..." ... /> element and
    # substitute only the range value. Joint elements are one line, self-closing.
    pat = re.compile(
        r'(<joint\b[^>]*\bname="' + re.escape(name) + r'"[^>]*\brange=")([^"]*)(")'
    )
    new_text, n = pat.subn(lambda m: m.group(1) + new_range + m.group(3), text)
    if n != 1:
        raise RuntimeError(f"expected exactly 1 range match for {name}, got {n}")
    return new_text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=MJCF)
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--dry-run", action="store_true", help="report table, do not write")
    ap.add_argument(
        "--verify",
        action="store_true",
        help="re-parse the MJCF and assert every range == URDF; do not write",
    )
    args = ap.parse_args()

    urdf = urdf_limits(args.urdf)
    mjcf = mjcf_hinge_ranges(args.mjcf)

    assert set(urdf) == set(mjcf), (
        f"joint-name mismatch: urdf-only={set(urdf)-set(mjcf)} "
        f"mjcf-only={set(mjcf)-set(urdf)}"
    )
    TOL = 1e-9

    if args.verify:
        bad = []
        for name, (ulo, uhi) in urdf.items():
            mlo, mhi = mjcf[name]
            if abs(mlo - ulo) > TOL or abs(mhi - uhi) > TOL:
                bad.append((name, (mlo, mhi), (ulo, uhi)))
        print(f"hinge joints checked: {len(mjcf)}")
        if bad:
            for name, mv, uv in bad:
                print(f"  MISMATCH {name}: mjcf={mv} urdf={uv}")
            raise SystemExit(f"VERIFY FAILED: {len(bad)} joints diverge from URDF")
        print("VERIFY OK: all ranges == URDF limits")
        return

    # Build the before/after table and rewrite.
    text = open(args.mjcf).read()
    ndiff = 0
    print(
        f"{'joint':24s} {'old lo':>11} {'new lo':>11} | {'old hi':>11} {'new hi':>11}  changed"
    )
    for name in mjcf:  # MJCF document order
        mlo, mhi = mjcf[name]
        ulo, uhi = urdf[name]
        changed = abs(mlo - ulo) > TOL or abs(mhi - uhi) > TOL
        ndiff += changed
        print(
            f"{name:24s} {mlo:11.5f} {ulo:11.5f} | {mhi:11.5f} {uhi:11.5f}  "
            f"{'YES' if changed else '.'}"
        )
        text = rewrite(text, name, ulo, uhi)

    print(f"\n{len(mjcf)} hinge joints total, {ndiff} diverged from URDF")
    if args.dry_run:
        print("--dry-run: not writing")
        return
    with open(args.mjcf, "w") as f:
        f.write(text)
    print(f"wrote {args.mjcf}")


if __name__ == "__main__":
    main()
