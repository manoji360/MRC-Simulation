import pytest

from builders import joint, link, robot, rover_like_items, ROVER_EXTRAS
from urdf_pipeline.errors import PipelineError
from urdf_pipeline.parse import is_xacro, load_urdf, parse_urdf_string, to_xml_string, validate


def test_valid_rover_like_has_no_problems():
    r = parse_urdf_string(robot("r", rover_like_items(), ROVER_EXTRAS))
    assert validate(r) == []
    assert len(r.extras) == 2


def test_multiple_roots_and_dangling_child_are_both_reported():
    # Mirrors the bug in the OSU repo's committed rover2_body.urdf.
    text = robot("bad", [link("base"), link("orphan"), link("camera1"),
                         joint("cam", "fixed", "base", "camera1"),
                         joint("cam2", "fixed", "base", "camera2")])
    problems = validate(parse_urdf_string(text))
    assert any("camera2" in p and "not defined" in p for p in problems)
    assert any("root links" in p and "orphan" in p for p in problems)


def test_loop_is_reported():
    text = robot("loop", [link("a"), link("b"), joint("ab", "fixed", "a", "b"), joint("ba", "fixed", "b", "a")])
    assert any("loop" in p for p in validate(parse_urdf_string(text)))


def test_duplicates_bad_type_and_bad_mimic():
    text = robot("dup", [link("a"), link("a"), link("b"),
                         joint("j", "hinge", "a", "b", mimic="nope")])
    problems = validate(parse_urdf_string(text))
    assert any("defined more than once" in p for p in problems)
    assert any("type 'hinge'" in p for p in problems)
    assert any("mimics joint 'nope'" in p for p in problems)


def test_xacro_detection_ignores_comments():
    assert is_xacro(b'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="x"/></robot>')
    assert not is_xacro(b'<robot name="r"><!-- <xacro:include filename="$(find p)/x"/> --><link name="a"/></robot>')


def test_load_errors_are_friendly(tmp_path):
    with pytest.raises(PipelineError, match="File not found"):
        load_urdf(tmp_path / "missing.urdf")
    bad = tmp_path / "bad.urdf"
    bad.write_text("<robot><link name='a'></robot>")
    with pytest.raises(PipelineError, match="not valid XML"):
        load_urdf(bad)
    sdf = tmp_path / "world.urdf"
    sdf.write_text("<sdf version='1.7'/>")
    with pytest.raises(PipelineError, match="expected <robot>"):
        load_urdf(sdf)


def test_write_then_read_is_stable():
    r = parse_urdf_string(robot("r", rover_like_items(), ROVER_EXTRAS))
    again = parse_urdf_string(to_xml_string(r))
    assert to_xml_string(again) == to_xml_string(r)
