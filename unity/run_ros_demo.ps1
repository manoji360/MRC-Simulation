<#
.SYNOPSIS
  End-to-end ROS 2 test: the ROS 2 waypoint follower (in WSL) drives the Unity rover over ROS-TCP.

.DESCRIPTION
  1. In WSL (once):   bash ros2/setup_ros2_bridge.sh
  2. In WSL:          source ~/mrc_ros_ws/install/setup.bash
                      ros2 launch mrc_waypoint_demo waypoint_demo.launch.py
  3. In Windows:      powershell -ExecutionPolicy Bypass -File unity\run_ros_demo.ps1
  Unity runs headless (batch mode) with the imported robot (default variant: tuned) and must be closed.
  Results: unity/MRC-Rover/FeasibilityResults/<variant>/ros.json
#>
param(
    [string]$Variant = "tuned",
    [string]$UnityPath
)
$ErrorActionPreference = "Stop"
$project = Join-Path $PSScriptRoot "MRC-Rover"
$results = Join-Path $project "FeasibilityResults"
if (-not $UnityPath) {
    $version = (Select-String -Path (Join-Path $project "ProjectSettings\ProjectVersion.txt") -Pattern "m_EditorVersion: (.+)").Matches[0].Groups[1].Value.Trim()
    $UnityPath = "C:\Program Files\Unity\Hub\Editor\$version\Editor\Unity.exe"
}
if (-not (Test-Path $UnityPath)) { throw "Unity not found at $UnityPath (pass -UnityPath)." }

$out = Join-Path $results $Variant
New-Item -ItemType Directory -Force $out | Out-Null
Remove-Item (Join-Path $out "ros.json") -ErrorAction SilentlyContinue
$xml = Join-Path $out "ros_tests.xml"
$log = Join-Path $out "ros_tests.log"
Write-Host "Running the ROS waypoint test in Unity (needs the ROS 2 launch running in WSL)..."
$unityArgs = @("-batchmode", "-nographics", "-projectPath", "`"$project`"", "-runTests", "-testPlatform", "PlayMode",
               "-testFilter", "RosWaypointTests", "-testResults", "`"$xml`"", "-variant", $Variant, "-ros", "-logFile", "`"$log`"")
Start-Process -FilePath $UnityPath -ArgumentList $unityArgs -Wait -NoNewWindow

$json = Join-Path $out "ros.json"
if (-not (Test-Path $json)) { Write-Host "No ros.json written - see $log" -ForegroundColor Red; exit 1 }
$r = Get-Content $json -Raw | ConvertFrom-Json
$result = if (Select-String -Path $xml -Pattern 'result="Passed"' -Quiet) { "PASS" } else { "FAIL" }
Write-Host ""
Write-Host "=== ROS 2 <-> Unity waypoint test ($Variant): $result ===" -ForegroundColor Cyan
Write-Host ("Connected: {0}   first /cmd_vel after {1} s   /cmd_vel received {2} ({3} Hz)   /odom published {4}" -f `
    $r.connected, $r.first_cmd_vel_after_s, $r.cmd_vel_received, $r.cmd_vel_rate_hz, $r.odom_published)
Write-Host ("Time {0} s   final odom position ({1})" -f $r.seconds, ($r.final_odom_xy -join ", "))
Write-Host "Node: $($r.node_status)"
Write-Host "Details: $json"
