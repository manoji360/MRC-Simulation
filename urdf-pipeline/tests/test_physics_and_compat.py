"""Tests of physics checks, Unity-unit conversion, compatibility fixes, frames, couplings, springs and limits."""

import math

import pytest

from builders import ARM_LIMIT, ROVER_EXTRAS, ROVER_LIKE_TOTAL_MASS, joint, link, robot, rover_like_items
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
    """The rover-like test robot, parsed."""
    return parse_urdf_string(robot("rover_like", rover_like_items(), ROVER_EXTRAS))


def test_joint_roles(rover):
    """Wheel, suspension, arm and passive roles are guessed correctly, and can be overridden."""
    roles = joint_roles(rover)
    assert {j for j, r in roles.items() if r == "wheel"} == {
        f"{s}_{e}_wheel_joint" for s in ("left", "right", "rear") for e in ("front", "back")}
    assert {j for j, r in roles.items() if r == "suspension"} == {"left_bogie_joint", "right_bogie_joint", "rear_bogie_joint"}
    assert roles["back_link_joint"] == "passive"
    assert roles["wrist_joint"] == "arm"  # continuous, but has an effort limit and no wheel name
    assert all(roles[f"arm_{i}_joint"] == "arm" for i in range(1, 6))
    assert joint_roles(rover, {"back_link_joint": "wheel"})["back_link_joint"] == "wheel"


def test_physics_issues_flag_real_rover_style_problems(rover):
    """physics_issues warns about massless shapes and free-swinging joints, and reports the total mass."""
    issues = physics_issues(rover)
    text = " ".join(m for _, m in issues)
    assert [level for level, m in issues if "jaws" in m] == ["warn"]
    assert "back_link_joint" in text and "swing freely" in text
    assert f"{ROVER_LIKE_TOTAL_MASS:.3f} kg" in text


def test_bad_inertia_is_flagged():
    """Inertia that breaks the triangle inequality is flagged as impossible."""
    bad = robot("r", ['<link name="a"><inertial><mass value="1"/>'
                      '<inertia ixx="1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link>'])
    assert any("impossible inertia" in m for _, m in physics_issues(parse_urdf_string(bad)))


def test_physics_block_converts_to_unity_units(rover):
    """physics_block converts joint limits to Unity degrees and keeps effort in N*m."""
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
    """Prismatic joint limits stay in metres."""
    text = robot("r", [link("a", 1), link("b", 1),
                       joint("slide", "prismatic", "a", "b", limit='lower="0" upper="0.3" effort="10" velocity="0.2"')])
    drive = physics_block(parse_urdf_string(text))["joints"]["slide"]["unity_drive"]
    assert drive["units"] == "metres" and drive["upper_limit"] == pytest.approx(0.3)


def test_default_axis_is_added_and_removed(rover):
    """A missing <axis> is written explicitly by prepare and removed again by rebuild."""
    original = rover.copy()
    assert joints_missing_axis(rover) == ["back_link_joint"]
    assert add_default_axes(rover) == ["back_link_joint"]
    assert rover.joints["back_link_joint"].element.find("axis").get("xyz") == "1 0 0"
    remove_added_axes(rover, ["back_link_joint"])
    assert diff_robots(original, rover) == []


def test_placeholder_inertials_are_added_and_removed(rover):
    """Placeholder masses are added to massless links and removed again."""
    original = rover.copy()
    missing = links_missing_inertial(rover)
    assert set(missing) == {"jaws"} | {f"camera_frame_{i}" for i in range(4)}
    add_placeholder_inertials(rover, 0.001)
    assert total_mass(rover) == pytest.approx(ROVER_LIKE_TOTAL_MASS + 0.005)
    remove_placeholder_inertials(rover, missing)
    assert diff_robots(original, rover) == []


def test_joint_couplings_from_config_and_mimic(rover):
    """Couplings from the config resolve same/opposite rotation from the axes; URDF mimic tags are read."""
    from urdf_pipeline.errors import PipelineError
    from urdf_pipeline.physics import joint_couplings

    # left and right bogies both turn about +Y: "opposite_rotation" -> -1, "same_rotation" -> +1
    c = joint_couplings(rover, {"right_bogie_joint": {"leader": "left_bogie_joint", "multiplier": "opposite_rotation"}})
    assert c == [{"follower": "right_bogie_joint", "leader": "left_bogie_joint", "multiplier": -1.0,
                  "offset": 0.0, "source": "config"}]
    assert joint_couplings(rover, {"rear_bogie_joint": {"leader": "left_bogie_joint",
                                                         "multiplier": "same_rotation"}})[0]["multiplier"] == 1.0
    with pytest.raises(PipelineError, match="parallel axes"):   # arm_1 turns about X, bogies about Y
        joint_couplings(rover, {"arm_1_joint": {"leader": "left_bogie_joint", "multiplier": "same_rotation"}})
    with pytest.raises(PipelineError, match="not a moving joint"):
        joint_couplings(rover, {"arm_mount": {"leader": "left_bogie_joint", "multiplier": 1}})

    mimic = parse_urdf_string(robot("m", [link("a", 1), link("b", 1), link("c", 1),
                                          joint("j1", "revolute", "a", "b", limit=ARM_LIMIT),
                                          '<joint name="j2" type="revolute"><parent link="a"/><child link="c"/>'
                                          '<axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/>'
                                          '<mimic joint="j1" multiplier="-0.5" offset="0.1"/></joint>']))
    (m,) = joint_couplings(mimic)
    assert m["follower"] == "j2" and m["multiplier"] == -0.5 and m["source"] == "urdf_mimic"
    assert m["offset"] == pytest.approx(math.degrees(0.1), abs=1e-6)  # radians -> Unity degrees


def test_frames_collapse_and_restore(rover):
    """Frame-only links are collapsed with correct offsets and restored exactly."""
    original = rover.copy()
    assert frame_only_links(rover) == [f"camera_frame_{i}" for i in range(4)]  # imu has mass, jaws has a shape
    entries = collapse_frames(rover)
    assert len(rover.links) == len(original.links) - 4
    assert entries[1]["origin"]["xyz"] == [0.0, 0.01, 0.0]       # ROS: 1 cm to the left of camera_link
    assert entries[1]["unity"]["position"] == [-0.01, 0.0, 0.0]  # Unity: left is -X
    restore_frames(rover, entries)
    assert diff_robots(original, rover) == []
    assert max_difference(world_transforms(original), world_transforms(rover)) == 0.0


def test_joint_springs_auto_from_weight():
    """'auto' spring stiffness/damping follow from the weight on the wheels and the lever arm."""
    from urdf_pipeline.errors import PipelineError
    from urdf_pipeline.physics import joint_springs

    # 10 kg body; a swing arm (pivot about X) whose wheel sits 0.2 m ahead (+Y) of the pivot, plus a fixed wheel.
    text = robot("s", [link("body", 8), link("arm", 0.5), link("arm_wheel", 0.5), link("rear_wheel", 1),
                       joint("swing", "revolute", "body", "arm", xyz="0 0 0", axis="1 0 0",
                             limit='lower="-0.3" upper="0.3" effort="10" velocity="1"'),
                       joint("arm_wheel_joint", "continuous", "arm", "arm_wheel", xyz="0 0.2 -0.1", axis="1 0 0"),
                       joint("rear_wheel_joint", "continuous", "body", "rear_wheel", xyz="0 -0.3 -0.1", axis="1 0 0")])
    r = parse_urdf_string(text)
    (s,) = joint_springs(r, {"swing": {"max_sag_deg": 2}})
    load = 10 * 9.81 / 2                      # 1 of 2 wheels below the joint
    assert s["auto_basis"]["lever_m"] == pytest.approx(0.2)
    # The wheel sits at (y=0.2, z=-0.1): the weight pushes the arm to -2 deg, where the lever is 0.2cos2 - 0.1sin2.
    sag = math.radians(2)
    assert s["auto_basis"]["sag_direction_deg"] == pytest.approx(-2)
    assert s["stiffness_nm_per_rad"] == pytest.approx(load * (0.2 * math.cos(sag) - 0.1 * math.sin(sag)) / sag, rel=1e-3)
    assert s["damping_nms_per_rad"] == pytest.approx(2 * math.sqrt(s["stiffness_nm_per_rad"] * (load / 9.81) * 0.04), rel=1e-3)
    fixed = joint_springs(r, {"swing": {"stiffness_nm_per_rad": 100, "damping_nms_per_rad": 5}})[0]
    assert (fixed["stiffness_nm_per_rad"], fixed["damping_nms_per_rad"], fixed["auto_basis"]) == (100, 5, None)
    with pytest.raises(PipelineError, match="not a revolute/continuous joint"):
        joint_springs(r, {"no_such_joint": {}})


def test_locked_joints_validated(rover):
    """locked_joints accepts moving joints and rejects others."""
    from urdf_pipeline.errors import PipelineError
    from urdf_pipeline.physics import locked_joints

    assert locked_joints(rover, ["back_link_joint"]) == ["back_link_joint"]
    with pytest.raises(PipelineError, match="not a moving joint"):
        locked_joints(rover, ["arm_mount"])


def test_drive_limits_config_and_wheel_warning(tmp_path):
    """Wheels without limits are detected; drive_limits validation and deceleration default work."""
    from urdf_pipeline.config import Config
    from urdf_pipeline.errors import PipelineError
    from urdf_pipeline.physics import drive_limits_block, unlimited_wheels

    rover = parse_urdf_string(robot("rover_like", rover_like_items(), ROVER_EXTRAS))
    assert len(unlimited_wheels(rover)) == 6
    assert drive_limits_block({"max_linear_accel_mps2": 2.0, "wheel_accel_rev_s2": None}) == {
        "max_linear_accel_mps2": 2.0, "max_linear_decel_mps2": 2.0}
    Config(drive_limits={"max_linear_speed_mps": 3.7, "controller_rate_hz": 30}).check()
    with pytest.raises(PipelineError, match="did you mean 'max_linear_accel_mps2'"):
        Config(drive_limits={"max_linear_accel": 2.0}).check()
    with pytest.raises(PipelineError, match="positive number"):
        Config(drive_limits={"max_linear_speed_mps": -1}).check()
