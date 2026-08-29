"""Vive Tracker pose reader via SteamVR / OpenVR, in ROS REP-103 frame.

Moved here from a standalone ~/vive_pose checkout: it is hardware pose input,
the same category as receiver/glove and receiver/quest, and consumers were
reaching it through a hardcoded absolute path.

    from whatslab.receiver.vive import ViveTracker
    t = ViveTracker()
    pos, R = t.read()          # pos: (3,) metres, R: (3,3) rotation
    t.close()

Frame: OpenVR (+X right, +Y up, -Z fwd) -> ROS REP-103 (+X fwd, +Y left, +Z up)
Measured: 481 Hz update, 100% valid-pose rate, static jitter sigma ~0.3-0.5 mm.

`openvr` is imported lazily so that importing this module never requires SteamVR
to be installed - callers that have no tracker still import the package fine.
"""
from __future__ import annotations

import numpy as np

# OpenVR -> ROS REP-103
M = np.array([[0., 0., -1.],
              [-1., 0., 0.],
              [0., 1., 0.]])


def _convert(p):
    m = p.mDeviceToAbsoluteTracking
    A = np.array([[m[r][c] for c in range(4)] for r in range(3)])
    return M @ A[:, 3], M @ A[:, :3] @ M.T


class ViveTracker:
    def __init__(self, serial=None):
        import openvr

        self._vr_mod = openvr
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.sys = openvr.VRSystem()
        self.idx = None
        self.lighthouses = []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            c = self.sys.getTrackedDeviceClass(i)
            if c == openvr.TrackedDeviceClass_TrackingReference:
                self.lighthouses.append(i)
            elif c == openvr.TrackedDeviceClass_GenericTracker:
                sn = self.sys.getStringTrackedDeviceProperty(
                    i, openvr.Prop_SerialNumber_String)
                if serial is None or sn == serial:
                    self.idx, self.serial = i, sn
        if self.idx is None:
            raise RuntimeError("Vive tracker not found - is vrserver running?")

    def read(self):
        """Return (pos, R) in ROS frame, or (None, None) if pose invalid."""
        vr = self._vr_mod
        ps = self.sys.getDeviceToAbsoluteTrackingPose(
            vr.TrackingUniverseStanding, 0, vr.k_unMaxTrackedDeviceCount)
        p = ps[self.idx]
        if not p.bPoseIsValid:
            return None, None
        return _convert(p)

    def lighthouse_poses(self):
        vr = self._vr_mod
        ps = self.sys.getDeviceToAbsoluteTrackingPose(
            vr.TrackingUniverseStanding, 0, vr.k_unMaxTrackedDeviceCount)
        return [_convert(ps[i]) for i in self.lighthouses if ps[i].bPoseIsValid]

    def close(self):
        self._vr_mod.shutdown()
