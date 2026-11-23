"""Tests of the physical-parameter registry, mass adjustment and joint limits."""

import math

import pytest

from builders import ROVER_LIKE_TOTAL_MASS, joint, link, robot, write_rover_like
from urdf_pipeline.config import Config
from urdf_pipeline.errors import PipelineError
from urdf_pipeline.merge import rebuild
from urdf_pipeline.parameters import fidelity_rows
from urdf_pipeline.parse import parse_urdf_string
from urdf_pipeline.physics import center_of_mass, joint_limits_block, total_mass
from urdf_pipeline.pipeline import check, prepare


def P(value, status="measured", unit=None, **extra):
    """A registry entry for tests: {value, status, source, uncertainty} (+ unit and extras)."""
    entry = {"value": value, "status": status, "source": "test", "uncertainty": "x"}
    if unit:
        entry["unit"] = unit
    return {**entry, **extra}


def test_registry_validation():
    """Unknown names, bad statuses, wrong units and malformed values are rejected with clear messages."""
    Config(physical_parameters={"total_mass_kg": P(50.0), "_note": "ignored"}).check()
    with pytest.raises(PipelineError, match="did you mean 'total_mass_kg'"):
        Config(physical_parameters={"total_mass": P(50.0)}).check()
    with pytest.raises(PipelineError, match="status must be one of"):
        Config(physical_parameters={"total_mass_kg": P(50.0, status="guessed")}).check()
    with pytest.raises(PipelineError, match="unit must be 'kg'"):
        Config(physical_parameters={"total_mass_kg": P(50.0, unit="lb")}).check()
    with pytest.raises(PipelineError, match=r"\[dx, dy, dz\]"):
        Config(physical_parameters={"center_of_mass_offset_m": P([0, 0])}).check()
    with pytest.raises(PipelineError, match="joint_limits"):
        Config(joint_limits={"j": {"max_velocity": -1}}).check()


def test_fidelity_rows_list_untrusted_first():
    """The fidelity table lists only untrusted parameters, least trusted first."""
    rows = fidelity_rows({"total_mass_kg": P(50.0, "cad"), "wheel_max_torque_nm": P(30.0, "assumed"),
                          "imu_rate_hz": P(10, "rover_code"), "gps_rate_hz": P(5, "measured")})
    assert [r["name"] for r in rows] == ["wheel_max_torque_nm", "total_mass_kg"]


def test_mass_scale_and_com_shift_are_exact_and_rebuild_restores(tmp_path):
    """total_mass_kg and center_of_mass_offset_m change the robot exactly as asked, and rebuild undoes it."""
    urdf = write_rover_like(tmp_path / "in")
    target, offset = ROVER_LIKE_TOTAL_MASS * 1.1, [0.0, 0.0, 0.05]
    config = Config(physical_parameters={"total_mass_kg": P(target), "center_of_mass_offset_m": P(offset)})
    out = tmp_path / "out"
    m = prepare(urdf, out, config).manifest

    original = parse_urdf_string(urdf.read_text())
    prepared = parse_urdf_string((out / m["outputs"][0]).read_text())
    placeholders = len(m["unity_compat"]["placeholder_inertial"]["links"]) * 0.001
    assert total_mass(prepared) - placeholders == pytest.approx(target, rel=1e-6)
    before, after = center_of_mass(original)["ros"], center_of_mass(prepared)["ros"]
    # placeholders (1 g each) barely move the centre of mass; the shift must be the requested 5 cm
    assert [a - b for a, b in zip(after, before)] == pytest.approx(offset, abs=2e-4)
    assert m["mass_adjustment"]["scale"] == pytest.approx(1.1)
    assert m["physics"]["total_mass_kg"] == pytest.approx(target, rel=1e-6)   # Unity sees the adjusted robot
    assert m["physics"]["unity_expected_mass_kg"] == pytest.approx(target + placeholders, rel=1e-6)
    assert rebuild(out).differences == []


def test_no_adjustment_when_registry_matches_urdf(tmp_path):
    """A registry that matches the URDF changes nothing, but its entries still appear in check's fidelity table."""
    urdf = write_rover_like(tmp_path / "in")
    config = Config(physical_parameters={"total_mass_kg": P(ROVER_LIKE_TOTAL_MASS, "cad"),
                                         "center_of_mass_offset_m": P([0, 0, 0], "cad")})
    m = prepare(urdf, tmp_path / "out", config).manifest
    assert m["mass_adjustment"] == {}
    assert m["physics"]["parameters"]["total_mass_kg"]["unit"] == "kg"   # unit filled in for Unity
    assert any("Fidelity: 2" in line for line in _check_lines(urdf, config))


def _check_lines(urdf, config):
    """The rendered check report lines for a URDF and config."""
    from urdf_pipeline.report import render_check
    return render_check(check(urdf, config), config)


def test_joint_limits_block_converts_to_unity_degrees():
    """joint_limits are converted to Unity degrees; unknown joints are rejected."""
    text = robot("r", [link("a", 1), link("b", 1),
                       joint("j", "revolute", "a", "b", limit='lower="-1" upper="1" effort="1" velocity="1"')])
    block = joint_limits_block(parse_urdf_string(text), {"j": {"max_velocity": 0.5, "max_acceleration": 2.0}})
    assert block["j"]["max_velocity_unity"] == pytest.approx(math.degrees(0.5))
    assert block["j"]["max_acceleration_unity"] == pytest.approx(math.degrees(2.0))
    with pytest.raises(PipelineError, match="not a moving joint"):
        joint_limits_block(parse_urdf_string(text), {"nope": {"max_velocity": 1}})
