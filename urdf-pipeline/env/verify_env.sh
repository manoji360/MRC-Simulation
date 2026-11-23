#!/usr/bin/env bash
# PASS/FAIL checklist for the URDF pipeline dev environment (run inside WSL).
# Exits non-zero if any required check fails.
#
# Usage: bash verify_env.sh [REPO_ROOT]
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${1:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
INPUT_DIR="$REPO_ROOT/urdf-pipeline/input"

fails=0
pass() { printf '  \033[32mPASS\033[0m  %s\n' "$1"; }
fail() { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; fails=$((fails+1)); }
warn() { printf '  \033[33mWARN\033[0m  %s\n' "$1"; }
check() { local desc="$1"; shift; if "$@" >/dev/null 2>&1; then pass "$desc"; else fail "$desc"; fi; }

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
if [[ -f "${ROVER_WS:-$HOME/rover_ws}/install/setup.bash" ]]; then
  set +u; source "${ROVER_WS:-$HOME/rover_ws}/install/setup.bash"; set -u
fi

echo "System"
. /etc/os-release
[[ "${VERSION_ID:-}" == "24.04" ]] && pass "Ubuntu 24.04 ($PRETTY_NAME)" || fail "Ubuntu 24.04 (found ${PRETTY_NAME:-unknown})"
grep -qi microsoft /proc/version && pass "Running under WSL" || warn "Not running under WSL"

if command -v wslinfo >/dev/null 2>&1; then
  mode="$(wslinfo --networking-mode 2>/dev/null || echo unknown)"
  [[ "$mode" == "mirrored" ]] && pass "WSL networking mode: mirrored" \
    || warn "WSL networking mode: $mode (set networkingMode=mirrored in %UserProfile%\\.wslconfig)"
else
  warn "wslinfo not available; cannot check networking mode (run 'wsl --update' on Windows)"
fi
[[ -n "${WAYLAND_DISPLAY:-}${DISPLAY:-}" ]] && pass "WSLg display available" || fail "WSLg display available (DISPLAY/WAYLAND_DISPLAY unset)"

echo "ROS 2"
[[ "${ROS_DISTRO:-}" == "jazzy" ]] && pass "ROS_DISTRO=jazzy" || fail "ROS_DISTRO=jazzy (found '${ROS_DISTRO:-unset}')"
check "ros2 CLI"                      command -v ros2
check "xacro"                         command -v xacro
check "check_urdf (liburdfdom-tools)" command -v check_urdf
check "rviz2"                         command -v rviz2
check "colcon"                        command -v colcon
check "realsense2_description"        ros2 pkg prefix realsense2_description
check "rover_urdf built"              ros2 pkg prefix rover_urdf
check "rover_arm_urdf built"          ros2 pkg prefix rover_arm_urdf
check "python3"                       command -v python3

echo "Generated input ($INPUT_DIR)"
if [[ -f "$INPUT_DIR/rover.urdf" ]]; then
  check "check_urdf input/rover.urdf" check_urdf "$INPUT_DIR/rover.urdf"
  report="$(python3 - "$INPUT_DIR" <<'PY'
import os, sys, xml.etree.ElementTree as ET
d = sys.argv[1]
r = ET.parse(os.path.join(d, "rover.urdf")).getroot()
links = [l.get("name") for l in r.findall("link")]
joints = r.findall("joint")
children = {j.find("child").get("link") for j in joints}
roots = [l for l in links if l not in children]
uris = sorted({m.get("filename") for m in r.iter("mesh") if m.get("filename")})
missing = [u for u in uris if u.startswith("package://")
           and not os.path.isfile(os.path.join(d, u[len("package://"):]))]
print(f"links={len(links)} joints={len(joints)} roots={roots} meshes={len(uris)} missing={len(missing)}")
for u in missing:
    print("MISSING " + u)
sys.exit(0 if len(roots) == 1 and not missing else 1)
PY
)"; rc=$?
  echo "        $report" | head -n 1
  [[ $rc -eq 0 ]] && pass "single root, all referenced meshes present" || { fail "single root, all referenced meshes present"; echo "$report" | tail -n +2; }
  [[ -f "$INPUT_DIR/SOURCE.txt" ]] && pass "SOURCE.txt present" || warn "SOURCE.txt missing"
else
  warn "input/rover.urdf not generated yet (run generate_input.sh)"
fi

echo
if [[ $fails -eq 0 ]]; then echo "All required checks passed."; else echo "$fails check(s) failed."; fi
exit $(( fails > 0 ))
