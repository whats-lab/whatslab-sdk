#!/usr/bin/env python3
import argparse
import os
import re
import shutil

import numpy as np


def load_proposal(path):
    z = np.load(path, allow_pickle=True)
    return {str(n): (float(l), float(h)) for n, l, h
            in zip(z["names"], z["lo"], z["hi"])}


def apply(urdf, side, limits, axes, dry):
    s = open(urdf).read()
    orig = s
    log = []
    for short, val in sorted(limits.items()):
        jn = "%s_%s" % (side, short)
        pat = re.compile(
            r'(<joint\s+name="%s"[^>]*>.*?<limit[^>]*?)lower="([-0-9.eE+]+)"'
            r'(\s+)upper="([-0-9.eE+]+)"' % re.escape(jn), re.S)
        m = pat.search(s)
        if not m:
            log.append(("한계 미발견", jn, "", ""))
            continue
        lo, hi = np.deg2rad(val[0]), np.deg2rad(val[1])
        old = (float(m.group(2)), float(m.group(4)))
        s = s[:m.start()] + ('%slower="%.4f"%supper="%.4f"'
                             % (m.group(1), lo, m.group(3), hi)) + s[m.end():]
        log.append(("한계", jn,
                    "[%+.1f,%+.1f]" % (np.degrees(old[0]), np.degrees(old[1])),
                    "[%+.1f,%+.1f]" % (val[0], val[1])))
    for short, v in sorted(axes.items()):
        jn = "%s_%s" % (side, short)
        pat = re.compile(r'(<joint\s+name="%s"[^>]*>.*?<axis\s+xyz=")([^"]+)(")'
                         % re.escape(jn), re.S)
        m = pat.search(s)
        if not m:
            log.append(("축 미발견", jn, "", ""))
            continue
        old = m.group(2)
        new = "%.6f %.6f %.6f" % tuple(v)
        s = s[:m.start()] + m.group(1) + new + m.group(3) + s[m.end():]
        log.append(("축", jn, old, new))
    if not dry and s != orig:
        shutil.copy2(urdf, urdf + ".bak")
        tmp = urdf + ".tmp"
        with open(tmp, "w") as f:
            f.write(s)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, urdf)
    return log, s != orig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--side", required=True)
    ap.add_argument("--limits", default=None, help="proposal npz (deg)")
    ap.add_argument("--axes", default=None, help="axis npz: names + xyz(N,3)")
    ap.add_argument("--only", nargs="*", default=None,
                    help="이 관절만 적용 (미지정 시 전부)")
    ap.add_argument("--apply", action="store_true", help="미지정 시 dry-run")
    args = ap.parse_args()

    lim = load_proposal(args.limits) if args.limits else {}
    ax = {}
    if args.axes:
        z = np.load(args.axes, allow_pickle=True)
        ax = {str(n): np.asarray(v, float) for n, v in zip(z["names"], z["xyz"])}
    if args.only:
        lim = {k: v for k, v in lim.items() if k in args.only}
        ax = {k: v for k, v in ax.items() if k in args.only}
    log, changed = apply(args.urdf, args.side, lim, ax, dry=not args.apply)
    print("%s  (%s)" % (args.urdf, "적용" if args.apply else "dry-run"))
    for kind, jn, a, b in log:
        print("  %-10s %-28s %-18s -> %s" % (kind, jn, a, b))
    print("변경 %s" % ("있음" if changed else "없음"))


if __name__ == "__main__":
    main()
