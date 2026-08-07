import logging
import os
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from scipy.spatial.transform import Rotation as ScipyR

from whatslab.paths import models_root

logger = logging.getLogger(__name__)


JOINT_ORDER = [
    'thumb_cmc0',
    'thumb_cmc1',
    'thumb_mcp',
    'thumb_ip',
    'index_mcp',
    'index_pip',
    'index_dip',
    'middle_mcp',
    'middle_pip',
    'middle_dip',
    'ring_mcp',
    'ring_pip',
    'ring_dip',
    'pinky_mcp',
    'pinky_pip',
    'pinky_dip',
]

URDF_TO_JOINT = {
    'wrist':        None,        
    'thumb_cmc0':   'thumb_cmc0',
    'thumb_cmc1':   'thumb_cmc1',
    'thumb_mcp':    'thumb_mcp',
    'thumb_ip':     'thumb_ip',
    'index_mcp':    'index_mcp',
    'index_pip':    'index_pip',
    'index_dip':    'index_dip',
    'middle_mcp':   'middle_mcp',
    'middle_pip':   'middle_pip',
    'middle_dip':   'middle_dip',
    'ring_mcp':     'ring_mcp',
    'ring_pip':     'ring_pip',
    'ring_dip':     'ring_dip',
    'pinky_mcp':    'pinky_mcp',
    'pinky_pip':    'pinky_pip',
    'pinky_dip':    'pinky_dip',
}


def _correct_quat(raw_xyzw: np.ndarray) -> np.ndarray:
    return np.array([raw_xyzw[0], -raw_xyzw[1], -raw_xyzw[2], raw_xyzw[3]])

def _parse_urdf_joints(urdf_path: str) -> dict:
    tree = ET.parse(urdf_path)
    joints = {}
    for j in tree.getroot().iter('joint'):
        origin = j.find('origin')
        if origin is not None:
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            joints[j.get('name')] = xyz
    return joints


def build_model(urdf_path: str, hand_type: str = 'left') -> 'pin.Model':

    joints = _parse_urdf_joints(urdf_path)
    pfx = hand_type + '_'
    model = pin.Model()
    UNIVERSE = 0

    def add_sph(parent_id: int, suffix: str, xyz) -> int:
        placement = pin.SE3(np.eye(3), np.array(xyz, dtype=float))
        jid = model.addJoint(
            parent_id,
            pin.JointModelSpherical(),
            placement,
            pfx + suffix,
        )
        model.appendBodyToJoint(
            jid,
            pin.Inertia(1e-3, np.zeros(3), np.eye(3) * 1e-6),
            pin.SE3.Identity(),
        )
        return jid

    cmc0_id  = add_sph(UNIVERSE, 'thumb_cmc0',  joints[f'{pfx}thumb_cmc0'])
    cmc1_id  = add_sph(cmc0_id,  'thumb_cmc1',  joints[f'{pfx}thumb_cmc1_x'])
    tmcp_id  = add_sph(cmc1_id,  'thumb_mcp',   joints[f'{pfx}thumb_mcp'])
    tip_id   = add_sph(tmcp_id,  'thumb_ip',    joints[f'{pfx}thumb_ip'])
    _        = add_sph(tip_id,   'thumb_tip',   joints[f'{pfx}thumb_tip']) 

    imcp_id = add_sph(UNIVERSE, 'index_mcp', joints[f'{pfx}index_mcp_z'])
    ipip_id = add_sph(imcp_id,  'index_pip', joints[f'{pfx}index_pip'])
    idip_id = add_sph(ipip_id,  'index_dip', joints[f'{pfx}index_dip'])
    _       = add_sph(idip_id,  'index_tip', joints[f'{pfx}index_tip']) 

    mmcp_id = add_sph(UNIVERSE, 'middle_mcp', joints[f'{pfx}middle_mcp_z'])
    mpip_id = add_sph(mmcp_id,  'middle_pip', joints[f'{pfx}middle_pip'])
    mdip_id = add_sph(mpip_id,  'middle_dip', joints[f'{pfx}middle_dip'])
    _       = add_sph(mdip_id,  'middle_tip', joints[f'{pfx}middle_tip']) 

    rmcp_id = add_sph(UNIVERSE, 'ring_mcp', joints[f'{pfx}ring_mcp_z'])
    rpip_id = add_sph(rmcp_id,  'ring_pip', joints[f'{pfx}ring_pip'])
    rdip_id = add_sph(rpip_id,  'ring_dip', joints[f'{pfx}ring_dip'])
    _       = add_sph(rdip_id,  'ring_tip', joints[f'{pfx}ring_tip']) 

    cmc_xyz  = np.array(joints[f'{pfx}pinky_0'])
    pmcp_xyz = np.array(joints[f'{pfx}pinky_mcp_z'])
    pmcp_id  = add_sph(UNIVERSE, 'pinky_mcp', (cmc_xyz + pmcp_xyz).tolist())
    ppip_id  = add_sph(pmcp_id,  'pinky_pip', joints[f'{pfx}pinky_pip'])
    pdip_id  = add_sph(ppip_id,  'pinky_dip', joints[f'{pfx}pinky_dip'])
    _        = add_sph(pdip_id,  'pinky_tip', joints[f'{pfx}pinky_tip']) 

    return model


class HandSphericalFK:

    def __init__(self, hand_type: str = 'left', urdf_path: str = None):

        self.hand_type = hand_type.lower()
        self.pfx = self.hand_type + '_'

        if urdf_path is None:
            urdf_path = os.path.join(models_root(), 'base_hand', 'urdf',
                                     f'{self.hand_type}.urdf')
        
        self.model = build_model(urdf_path, self.hand_type)
        self.data  = self.model.createData()

        self.name_to_jid = {
            self.model.names[i]: i
            for i in range(1, self.model.njoints)
        }

        self.sensor_to_jid = {
            s_idx + 1: self.name_to_jid[self.pfx + suffix]
            for s_idx, suffix in enumerate(JOINT_ORDER)
        }

        self._joint_map = {
            1: self.pfx + 'thumb_cmc0', 2: self.pfx + 'thumb_cmc1', 3: self.pfx + 'thumb_mcp',
            4: self.pfx + 'thumb_ip',   5: self.pfx + 'thumb_tip',
            6: self.pfx + 'index_mcp',  7: self.pfx + 'index_pip',  8: self.pfx + 'index_dip',  9: self.pfx + 'index_tip',
            10: self.pfx + 'middle_mcp',11: self.pfx + 'middle_pip', 12: self.pfx + 'middle_dip',13: self.pfx + 'middle_tip',
            14: self.pfx + 'ring_mcp',  15: self.pfx + 'ring_pip',   16: self.pfx + 'ring_dip',  17: self.pfx + 'ring_tip',
            18: self.pfx + 'pinky_mcp', 19: self.pfx + 'pinky_mcp',  20: self.pfx + 'pinky_pip', 21: self.pfx + 'pinky_dip',
            22: self.pfx + 'pinky_tip',
        }

        logger.info("[HandSphericalFK] %s hand loaded (njoints=%d, nq=%d)",
                    hand_type, self.model.njoints, self.model.nq)
        logger.debug("  joints: %s", list(self.model.names[1:]))

    def sensor_to_q(self, sensor_quats_17: np.ndarray) -> np.ndarray:
        q = pin.neutral(self.model)   
        sign = 1 if self.hand_type == 'right' else -1 
            
        for sensor_idx, jid in self.sensor_to_jid.items():
            raw       = sensor_quats_17[sensor_idx]
            corrected = _correct_quat(raw)
            if self.hand_type == 'right':
                corrected[1] = -corrected[1]
                corrected[0] = -corrected[0]
            corrected = corrected / (np.linalg.norm(corrected) + 1e-9) 


            if sensor_idx == 14:
                _PINKY_CMC_OFFSET = ScipyR.from_euler('y', sign * np.radians(20.0))
                corrected = (_PINKY_CMC_OFFSET * ScipyR.from_quat(corrected)).as_quat()

            idx_q = self.model.idx_qs[jid]
            q[idx_q: idx_q + 4] = corrected   

        return q

    def compute(self, sensor_quats_17: np.ndarray):
        q = self.sensor_to_q(sensor_quats_17)
        pin.forwardKinematics(self.model, self.data, q)

        positions = {}
        rotations = {}
        for jid in range(1, self.model.njoints):
            name = self.model.names[jid]
            oMi  = self.data.oMi[jid]
            positions[name] = oMi.translation.copy()
            rotations[name] = oMi.rotation.copy()

        return q, positions, rotations

    def compute_positions(self, sensor_quats_17: np.ndarray) -> np.ndarray:
        _, pos_dict, _ = self.compute(sensor_quats_17)
        out = np.zeros((23, 3), dtype=np.float64)
        for i, name in self._joint_map.items():
            if name in pos_dict:
                out[i] = pos_dict[name]
        return out


class HandRerunViz:

    def __init__(self, hand_type: str, urdf_path: str, entity_prefix: str = 'hand'):
        self.fk            = HandSphericalFK(hand_type, urdf_path)
        self.urdf_path     = urdf_path
        self.entity_prefix = entity_prefix
        pfx                = hand_type + '_'

        self.link_to_jname = {}
        for link_suf, joint_suf in URDF_TO_JOINT.items():
            if joint_suf is None:
                continue
            self.link_to_jname[pfx + link_suf] = pfx + joint_suf

    def setup(self):
        try:
            import rerun as rr
        except ImportError as e:
            raise ImportError(f"pip install rerun-sdk: {e}")

        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_X_UP, static=True)

        rr.log(
            "world/origin",
            rr.Arrows3D(
                vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                labels=["X", "Y", "Z"]
            ),
            static=True
        )

        rr.log(
            "world/camera",
            rr.Transform3D(
                translation=[0.2, -0.2, 0.2],
                rotation=rr.Quaternion(xyzw=[0.4, 0.2, 0.1, 0.9])
            ),
            static=True
        )

        pfx  = self.fk.pfx
        tree = ET.parse(self.urdf_path)
        n_logged = 0

        for visual in tree.getroot().iter('visual'):
            vname     = visual.get('name', '')
            link_name = vname.replace('_visual', '')

            is_wrist  = (link_name == pfx + 'wrist')
            is_mapped = (link_name in self.link_to_jname)
            if not is_wrist and not is_mapped:
                continue

            mesh_elem = visual.find('geometry/mesh')
            if mesh_elem is None:
                continue

            filename = mesh_elem.get('filename', '')
            filename = filename.replace('package://dexhand_description/',
                                        models_root() + '/')
            if not os.path.exists(filename):
                logger.warning("메쉬 없음: %s", filename)
                continue

            scale = [float(s) for s in mesh_elem.get('scale', '1 1 1').split()]

            origin = visual.find('origin')
            if origin is not None:
                vis_xyz  = [float(v) for v in origin.get('xyz', '0 0 0').split()]
                vis_rpy  = [float(v) for v in origin.get('rpy', '0 0 0').split()]
                vis_quat = ScipyR.from_euler('xyz', vis_rpy).as_quat()
            else:
                vis_xyz  = [0., 0., 0.]
                vis_quat = [0., 0., 0., 1.]

            link_entity   = f'{self.entity_prefix}/{link_name}'
            visual_entity = f'{link_entity}/visual'

            rr.log(visual_entity, rr.Transform3D(
                translation=vis_xyz,
                rotation=rr.Quaternion(xyzw=vis_quat),
                scale=scale,
            ), static=True)

            rr.log(visual_entity, rr.Asset3D(path=filename), static=True)

            if is_wrist:
                rr.log(link_entity, rr.Transform3D(
                    translation=[0., 0., 0.],
                    rotation=rr.Quaternion(xyzw=[0., 0., 0., 1.]),
                ), static=True)

            axis_length = 0.01
            rr.log(
                f"{link_entity}/axes",
                rr.Arrows3D(
                    vectors=[[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                ),
                static=True
            )

            n_logged += 1

        logger.info("[HandRerunViz] 메쉬 등록 완료 (%d개 링크, 손목 포함)", n_logged)

    def update(self, sensor_quats_17: np.ndarray, timestamp: float = None):
        try:
            import rerun as rr
        except ImportError:
            raise ImportError("pip install rerun-sdk")

        if timestamp is not None:
            rr.set_time('time', timestamp=timestamp)

        _, positions, rotations = self.fk.compute(sensor_quats_17)

        for link_name, joint_name in self.link_to_jname.items():
            if joint_name not in positions:
                continue

            pos  = positions[joint_name]
            quat = ScipyR.from_matrix(rotations[joint_name]).as_quat()

            rr.log(
                f'{self.entity_prefix}/{link_name}',
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=quat),
                ),
            )
