import math

import pytest

from builders import ROVER_EXTRAS, ROVER_LIKE_TOTAL_MASS, joint, link, robot, rover_like_items
from urdf_pipeline.compat import (
    add_default_axes, add_placeholder_inertials, joints_missing_axis, links_missing_inertial,
    remove_added_axes, remove_placeholder_inertials,
)
from urdf_pipeline.compare import diff_robots
from urdf_pipeline.frames import collapse_frames, frame_only_links, restore_frames
from urdf_pipeline.kinematics import max_difference, world_transforms
from urdf_pipeline.parse import parse_urdf_string
from urdf_pipeline.physics import joint_roles, physics_block, physics_issues, total_mass


@pytest.fixture
def rover():
    return parse_urdf_string(robot("rover_like", rover_like_items(), ROVER_EXTRAS))


def test_joint_roles(rover):
    roles = joint_roles(rover)
    assert {j for j, r in roles.items() if r == "wheel"} == {
        f"{s}_{e}_wheel_joint" for s in ("left", "right", "rear") for e in ("front", "back")}
    assert {j for j, r in roles.items() if r == "suspension"} == {"left_bogie_joint", "right_bogie_joint", "rear_bogie_joint"}
    assert roles["back_link_joint"] == "passive"
    assert roles["wrist_joint"] == "arm"  # continuous, but has an effort limit and no wheel name
    assert all(roles[f"arm_{i}_joint"] == "arm" for i in range(1, 6))
    assert joint_roles(rover, {"back_link_joint": "wheel"})["back_link_joint"] == "wheel"


def test_physics_issues_flag_real_rover_style_problems(rover):
    issues = physics_issues(rover)
    text = " ".join(m for _, m in issues)
    assert [level for level, m in issues if "jaws" in m] == ["warn"]
    assert "back_link_joint" in text and "swing freely" in text
    assert f"{ROVER_LIKE_TOTAL_MASS:.3f} kg" in text


def test_bad_inertia_is_flagged():
    bad = robot("r", ['<link name="a"><inertial><mass value="1"/>'
                      '<inertia ixx="1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link>'])
    assert any("impossible inertia" in m for _, m in physics_issues(parse_urdf_string(bad)))


def test_physics_block_converts_to_unity_units(rover):
    block = physics_block(rover)
    assert block["total_mass_kg"] == pytest.approx(ROVER_LIKE_TOTAL_MASS)
    arm = block["joints"]["arm_1_joint"]["unity_drive"]
    assert arm["units"] == "degrees"
    assert arm["lower_limit"] == pytest.approx(math.degrees(-1.5))
    assert arm["max_velocity"] == pytest.approx(math.degrees(1.0))
    assert arm["force_limit"] == 50
    wheel = block["joints"]["left_front_wheel_joint"]["unity_drive"]
    assert wheel["lower_limit"] is None and wheel["upper_limit"] is None
    assert block["links_without_mass"] == ["jaws"]


def test_prismatic_stays_in_metres():
    text = robot("r", [link("a", 1), link("b", 1),
                       joint("slide", "prismatic", "a", "b", limit='lower="0" upper="0.3" effort="10" velocity="0.2"')])
    drive = physics_block(parse_urdf_string(text))["joints"]["slide"]["unity_drive"]
    assert drive["units"] == "metres" and drive["upper_limit"] == pytest.approx(0.3)


def test_default_axis_is_added_and_removed(rover):
    original = rover.copy()
    assert joints_missing_axis(rover) == ["back_link_joint"]
    assert add_default_axes(rover) == ["back_link_joint"]
    assert rover.joints["back_link_joint"].element.find("axis").get("xyz") == "1 0 0"
    remove_added_axes(rover, ["back_link_joint"])
    assert diff_robots(original, rover) == []


def test_placeholder_inertials_are_added_and_removed(rover):
    original = rover.copy()
    missing = links_missing_inertial(rover)
    assert set(missing) == {"jaws"} | {f"camera_frame_{i}" for i in range(4)}
    add_placeholder_inertials(rover, 0.001)
    assert total_mass(rover) == pytest.approx(ROVER_LIKE_TOTAL_MASS + 0.005)
    remove_placeholder_inertials(rover, missing)
    assert diff_robots(original, rover) == []


def test_frames_collapse_and_restore(rover):
    original = rover.copy()
    assert frame_only_links(rover) == [f"camera_frame_{i}" for i in range(4)]  # imu has mass, jaws has a shape
    entries = collapse_frames(rover)
    assert len(rover.links) == len(original.links) - 4
    assert entries[1]["origin"]["xyz"] == [0.0, 0.01, 0.0]       # ROS: 1 cm to the left of camera_link
    assert entries[1]["unity"]["position"] == [-0.01, 0.0, 0.0]  # Unity: left is -X
    restore_frames(rover, entries)
    assert diff_robots(original, rover) == []
    assert max_difference(world_transforms(original), world_transforms(rover)) == 0.0
