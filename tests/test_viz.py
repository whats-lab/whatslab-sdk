"""viz — 사람 손 뼈대 구성(_bone_pairs) 검증.

scene.py 는 시각화 모듈이라 pinocchio/trimesh/viser 를 필수로 top-import 한다.
따라서 이 테스트는 그 deps 가 있는 env 에서만 의미가 있어 importorskip 으로 게이팅
(없는 env=dex_mj 에서는 수집 에러 대신 깔끔히 skip).
"""
import pytest

pytest.importorskip("pinocchio")
pytest.importorskip("trimesh")
pytest.importorskip("viser")

from whatslab.viz.scene import _bone_pairs  # noqa: E402


def test_bone_pairs_from_skeleton():
    pairs = _bone_pairs()
    # 23 노드, root(wrist) 제외 22 개 부모-자식 뼈대
    assert len(pairs) == 22
    for parent_i, child_i in pairs:
        assert 0 <= parent_i < 23 and 0 <= child_i < 23
        assert parent_i != child_i


def test_viz_module_imports_without_viser():
    # scene 모듈은 viser 미설치 환경에서도 import 되어야 (heavy dep 는 lazy)
    import whatslab.viz  # noqa: F401
    assert "RobotArmViz" in whatslab.viz.__all__
