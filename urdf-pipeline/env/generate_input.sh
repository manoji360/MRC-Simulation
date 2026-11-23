#!/usr/bin/env bash
# Generate urdf-pipeline/input/ from the OSU Robotics Club rover repository.
#
#  1. clone/update OSURoboticsClub/Rover_2026_2027 in ~/rover_ws/src
#  2. build only the description packages (rover_urdf, rover_arm_urdf)
#  3. expand the top-level xacro -> rover.urdf
#  4. validate with check_urdf
#  5. copy rover.urdf + every mesh it references into urdf-pipeline/input/
#
# Usage (inside WSL, after setup_ros_jazzy.sh):
#   bash generate_input.sh [REPO_ROOT] [ATTACHMENT]
#     REPO_ROOT   MRC-Simulation checkout (default: two levels above this script)
#     ATTACHMENT  arm | northrop_arm | science   (default: arm)
#
# Environment overrides:
#   ROVER_WS     workspace dir            (default: ~/rover_ws)
#   ROVER_REF    branch/tag/commit to use (default: main)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${1:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
ATTACHMENT="${2:-arm}"
ROVER_WS="${ROVER_WS:-$HOME/rover_ws}"
ROVER_REF="${ROVER_REF:-main}"
UPSTREAM_URL="https://github.com/OSURoboticsClub/Rover_2026_2027.git"

INPUT_DIR="$REPO_ROOT/urdf-pipeline/input"
SRC_DIR="$ROVER_WS/src/Rover_2026_2027"
PKGS="$SRC_DIR/software/ros_packages"

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
die() { echo "ERROR: $*" >&2; exit 1; }

[[ -d "$REPO_ROOT/urdf-pipeline" ]] || die "REPO_ROOT '$REPO_ROOT' does not contain urdf-pipeline/"
[[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy not found. Run setup_ros_jazzy.sh first."

# ROS setup scripts reference unset variables; relax -u while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

# 1. Clone or update the rover repository.
mkdir -p "$ROVER_WS/src"
if [[ -d "$SRC_DIR/.git" ]]; then
  log "Updating rover repository ($ROVER_REF)"
  git -C "$SRC_DIR" fetch --depth 1 origin "$ROVER_REF"
  git -C "$SRC_DIR" checkout -q FETCH_HEAD
else
  log "Cloning rover repository ($ROVER_REF)"
  git clone --depth 1 --branch "$ROVER_REF" "$UPSTREAM_URL" "$SRC_DIR" \
    || { git clone "$UPSTREAM_URL" "$SRC_DIR" && git -C "$SRC_DIR" checkout -q "$ROVER_REF"; }
fi
UPSTREAM_SHA="$(git -C "$SRC_DIR" rev-parse HEAD)"
echo "Upstream commit: $UPSTREAM_SHA"

# 2. Build only what xacro needs to resolve $(find ...) and package://:
#    - rover_urdf, rover_arm_urdf: ament_cmake packages that install urdf/ (+ ros2_control/)
#    rover2_arm (MoveIt config) and rover2_control (drive code) are NOT built; their config
#    files are passed to xacro as explicit source-tree paths below.
BUILD_PKGS=(rover_urdf rover_arm_urdf)
log "Building ${BUILD_PKGS[*]}"
( cd "$ROVER_WS" && colcon build --packages-select "${BUILD_PKGS[@]}" --event-handlers console_cohesion+ )
set +u
# shellcheck disable=SC1091
source "$ROVER_WS/install/setup.bash"
set -u

# 2b. rover_body.ros2_control.xacro has `$(find rover2_control)` in a macro parameter
#     default, which xacro evaluates even when the caller overrides it. Building
#     rover2_control would drag in its message package (rosidl), so instead register a
#     stub in the ament index: a marker file + its config yaml. That is all `find` needs.
STUB_PREFIX="$ROVER_WS/stub_install"
rm -rf "$STUB_PREFIX"
mkdir -p "$STUB_PREFIX/share/ament_index/resource_index/packages" "$STUB_PREFIX/share/rover2_control/config"
touch "$STUB_PREFIX/share/ament_index/resource_index/packages/rover2_control"
cp "$PKGS/rover2_control/config/rover_drive.yaml" "$STUB_PREFIX/share/rover2_control/config/"
export AMENT_PREFIX_PATH="$STUB_PREFIX:${AMENT_PREFIX_PATH:-}"

# 3. Expand the top-level xacro.
TOP_XACRO="$PKGS/rover2_arm/config/rover.urdf.xacro"
[[ -f "$TOP_XACRO" ]] || die "top-level xacro not found: $TOP_XACRO"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

XACRO_ARGS=(
  "attachment:=$ATTACHMENT"
  "ros2_control_hardware_type:=main"
  "initial_positions_file:=$PKGS/rover2_arm/config/initial_positions.yaml"
  "drive_initial_positions_file:=$PKGS/rover_urdf/ros2_control/initial_positions.yaml"
  "ros2_control_config_path:=$PKGS/rover2_control/config/rover_drive.yaml"
)
log "Running xacro (attachment=$ATTACHMENT)"
xacro "$TOP_XACRO" "${XACRO_ARGS[@]}" > "$WORK/rover.urdf"

# 4. Validate.
log "Validating with check_urdf"
check_urdf "$WORK/rover.urdf" > "$WORK/check_urdf.txt" || { cat "$WORK/check_urdf.txt"; die "check_urdf failed"; }
head -n 3 "$WORK/check_urdf.txt"

# 5. Copy the URDF and every referenced mesh into input/, keeping the
#    <package>/<path> layout so package:// URIs map 1:1 onto input/.
log "Copying URDF and referenced meshes into $INPUT_DIR"
rm -rf "$INPUT_DIR"
mkdir -p "$INPUT_DIR"
cp "$WORK/rover.urdf" "$INPUT_DIR/rover.urdf"

python3 - "$WORK/rover.urdf" > "$WORK/meshes.txt" <<'PY'
import sys, xml.etree.ElementTree as ET
uris = {m.get("filename") for m in ET.parse(sys.argv[1]).getroot().iter("mesh")}
for u in sorted(u for u in uris if u):
    print(u)
PY

missing=0; copied=0
while IFS= read -r uri; do
  [[ "$uri" == package://* ]] || { echo "  skip (not package://): $uri"; continue; }
  rel="${uri#package://}"; pkg="${rel%%/*}"; path="${rel#*/}"
  share="$(ros2 pkg prefix --share "$pkg" 2>/dev/null)" || { echo "  MISSING package: $pkg"; missing=$((missing+1)); continue; }
  src="$share/$path"
  if [[ -f "$src" ]]; then
    mkdir -p "$INPUT_DIR/$pkg/$(dirname "$path")"
    cp "$src" "$INPUT_DIR/$pkg/$path"
    copied=$((copied+1))
  else
    echo "  MISSING file: $uri"; missing=$((missing+1))
  fi
done < "$WORK/meshes.txt"

counts="$(python3 - "$INPUT_DIR/rover.urdf" <<'PY'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
print(f"links={len(r.findall('link'))} joints={len(r.findall('joint'))}")
PY
)"

cat > "$INPUT_DIR/SOURCE.txt" <<EOF
upstream:   $UPSTREAM_URL
ref:        $ROVER_REF
commit:     $UPSTREAM_SHA
generated:  $(date -u +%Y-%m-%dT%H:%M:%SZ)
xacro:      software/ros_packages/rover2_arm/config/rover.urdf.xacro
xacro args: ${XACRO_ARGS[*]}
model:      $counts
meshes:     $copied copied, $missing missing
EOF

log "Summary"
cat "$INPUT_DIR/SOURCE.txt"
[[ $missing -eq 0 ]] || die "$missing referenced mesh(es) could not be found"
echo
echo "input/ is ready. Next: bash \"$SCRIPT_DIR/verify_env.sh\""
