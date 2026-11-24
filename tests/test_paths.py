import sys
import types

from whatslab import paths


def _fake_description(monkeypatch, share, cache):
    module = types.SimpleNamespace(
        get_share=lambda: str(share),
        hand_dir=lambda hand: cache / hand if hand == "leap_hand" else None,
    )
    monkeypatch.setitem(sys.modules, "dexhand_description", module)
    monkeypatch.delenv("WHATSLAB_MODELS_ROOT", raising=False)


def test_mesh_urdf_maps_share_urdf_to_fetched_hand(tmp_path, monkeypatch):
    _fake_description(monkeypatch, tmp_path / "share", tmp_path / "cache")
    urdf = tmp_path / "share" / "leap_hand" / "urdf" / "right.urdf"
    expected = tmp_path / "cache" / "leap_hand" / "urdf" / "right.urdf"
    assert paths.mesh_urdf(str(urdf)) == str(expected)


def test_mesh_urdf_keeps_path_when_fetch_fails_or_outside_share(tmp_path, monkeypatch):
    _fake_description(monkeypatch, tmp_path / "share", tmp_path / "cache")
    failed = str(tmp_path / "share" / "orca_hand_v2" / "orca_hand_v2_right.urdf")
    outside = str(tmp_path / "mine" / "hand.urdf")
    assert paths.mesh_urdf(failed) == failed
    assert paths.mesh_urdf(outside) == outside


def test_mesh_urdf_respects_models_root_override(tmp_path, monkeypatch):
    _fake_description(monkeypatch, tmp_path / "share", tmp_path / "cache")
    monkeypatch.setenv("WHATSLAB_MODELS_ROOT", str(tmp_path / "share"))
    urdf = str(tmp_path / "share" / "leap_hand" / "right.urdf")
    assert paths.mesh_urdf(urdf) == urdf
