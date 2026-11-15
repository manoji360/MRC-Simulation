#!/usr/bin/env bash
# Unity physics-backend feasibility test (Ubuntu / macOS version of run_feasibility.ps1).
#   URDF -> urdf_pipeline prepare -> Unity batch import -> Unity PlayMode physics tests
#
# Usage: bash unity/run_feasibility.sh [variant ...]
#   variants: default collapsed split-meshes convex-unity   (default: default collapsed split-meshes)
# Environment:
#   URDF=/path/robot.urdf     (default: urdf-pipeline/input/rover.urdf)
#   UNITY_PATH=/path/Unity    (default: Unity Hub install of the project's editor version)
#   PYTHON=python3
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
PROJECT="$HERE/MRC-Rover"
RESULTS="$PROJECT/FeasibilityResults"
URDF="${URDF:-$REPO/urdf-pipeline/input/rover.urdf}"
PYTHON="${PYTHON:-python3}"
VARIANTS=("$@")
[[ ${#VARIANTS[@]} -eq 0 ]] && VARIANTS=(default collapsed split-meshes)

VERSION="$(sed -n 's/^m_EditorVersion: //p' "$PROJECT/ProjectSettings/ProjectVersion.txt" | tr -d '\r')"
if [[ -z "${UNITY_PATH:-}" ]]; then
  for candidate in "$HOME/Unity/Hub/Editor/$VERSION/Editor/Unity" \
                   "/Applications/Unity/Hub/Editor/$VERSION/Unity.app/Contents/MacOS/Unity"; do
    [[ -x "$candidate" ]] && UNITY_PATH="$candidate" && break
  done
fi
[[ -x "${UNITY_PATH:-}" ]] || { echo "Unity $VERSION not found; set UNITY_PATH." >&2; exit 1; }
[[ -f "$URDF" ]] || { echo "URDF not found: $URDF (set URDF=...)" >&2; exit 1; }
export PYTHONPATH="$REPO/urdf-pipeline"

unity() { "$UNITY_PATH" -batchmode -nographics -projectPath "$PROJECT" "$@"; }

printf '\n%-14s %-40s %s\n' VARIANT RESULT "MASS (unity/expected)  DRIVE(m)  ARM_ERR(deg)  RT_FACTOR"
for variant in "${VARIANTS[@]}"; do
  case "$variant" in
    default)      prep=();                        convex=vhacd ;;
    collapsed)    prep=(--collapse-frames always); convex=vhacd ;;
    split-meshes) prep=(--split-meshes);           convex=vhacd ;;
    convex-unity) prep=();                        convex=unity ;;
    *) echo "Unknown variant: $variant" >&2; continue ;;
  esac
  out="$RESULTS/$variant"; prepared="$REPO/urdf-pipeline/output/$variant"
  mkdir -p "$out"
  "$PYTHON" -m urdf_pipeline prepare "$URDF" -o "$prepared" "${prep[@]}" > "$out/prepare.log" 2>&1 \
    || { printf '%-14s %s\n' "$variant" "prepare failed ($out/prepare.log)"; continue; }
  unity -executeMethod RoverImport.ImportFromCommandLine -prepared "$prepared" -variant "$variant" \
        -convex "$convex" -logFile "$out/import.log" \
    || { printf '%-14s %s\n' "$variant" "import failed ($out/import.log)"; continue; }
  unity -runTests -testPlatform PlayMode -testResults "$out/tests.xml" -variant "$variant" -logFile "$out/tests.log"
  "$PYTHON" - "$out" "$variant" <<'PY'
import json, sys, xml.etree.ElementTree as ET
from pathlib import Path
out, variant = Path(sys.argv[1]), sys.argv[2]
cases = {c.get("name"): c.get("result") for c in ET.parse(out / "tests.xml").iter("test-case")} if (out / "tests.xml").exists() else {}
failed = [n for n, r in cases.items() if r != "Passed"]
result = "tests did not run" if not cases else ("ALL PASS" if not failed else "FAIL: " + ", ".join(failed))
p = json.loads((out / "physics.json").read_text()) if (out / "physics.json").exists() else {}
get = lambda *k: (lambda d: [d := (d or {}).get(x) for x in k][-1])(p)
print(f"{variant:<14} {result:<40} {get('mass','unity_total_kg')}/{get('mass','unity_expected_kg')}  "
      f"{get('drive','distance_forward_m')}  {get('arm','worst_error_deg')}  {get('performance','real_time_factor')}")
if (out / "terrain.json").exists():
    t = json.loads((out / "terrain.json").read_text())
    lanes = t.get("lanes", {})
    crossed = [n for n, v in lanes.items() if v.get("crossed")]
    blocked = [n for n in lanes if n not in crossed]
    d = t.get("desert", {})
    print(f"{'':<14} terrain: crossed {len(crossed)} of {len(lanes)}" + (f" (blocked: {', '.join(blocked)})" if blocked else "")
          + (f"; desert {d.get('progress_m')} m in {d.get('seconds')} s, max tilt {d.get('max_tilt_deg')} deg" if d else ""))
PY
done
echo "Details: $RESULTS/<variant>/physics.json and import.json"
