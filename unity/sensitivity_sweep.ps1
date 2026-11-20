<#
.SYNOPSIS
  Which unmeasured rover parameters change the simulation's results the most?

.DESCRIPTION
  Runs the full PlayMode suite (physics + terrain tests) once with the registry values (baseline), then
  once at a low and a high value of each uncertain parameter, passed to Unity as -param name=value
  (see Assets/Scripts/Feasibility/SimParameters.cs). Results go to FeasibilityResults/sensitivity/<run>/.
  Prints, per parameter, how much each key metric moved, and ranks the parameters: measure the top ones
  on the real rover first (docs/rover-measurement-guide.md).
  Needs an imported robot (run_feasibility.ps1) and Unity closed. About 9 minutes per run.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File unity\sensitivity_sweep.ps1
  powershell -ExecutionPolicy Bypass -File unity\sensitivity_sweep.ps1 -Only tyre_mu_all,wheel_max_torque_nm
  powershell -ExecutionPolicy Bypass -File unity\sensitivity_sweep.ps1 -ReportOnly     # re-print the table
#>
param(
    [string[]]$Only,
    [string]$Variant = "tuned",
    [string]$UnityPath,
    [switch]$ReportOnly
)
$ErrorActionPreference = "Stop"
if ($Only) { $Only = @($Only | ForEach-Object { $_ -split "," } | ForEach-Object { $_.Trim() } | Where-Object { $_ }) }

$project = Join-Path $PSScriptRoot "MRC-Rover"
$results = Join-Path $project "FeasibilityResults"
$sweepDir = Join-Path $results "sensitivity"

# Parameter -> low / high value (as passed to -param). Ranges = the registry's stated uncertainty.
$sweep = [ordered]@{
    "total_mass_kg"             = @{ Lo = "45.55";       Hi = "55.67";      Why = "mass +/-10 %" }
    "center_of_mass_offset_m"   = @{ Lo = "0,0,-0.05";   Hi = "0,0,0.05";   Why = "CoM height +/-5 cm" }
    "tyre_mu_all"               = @{ Lo = "0.4";         Hi = "0.9";        Why = "tyre friction 0.4-0.9" }
    "wheel_max_torque_nm"       = @{ Lo = "10";          Hi = "50";         Why = "wheel torque cap 10-50 N*m" }
    "controller_wheel_radius_m" = @{ Lo = "0.146";       Hi = "0.154";      Why = "wheel radius 0.146-0.154 m" }
    "passive_joint_friction_nm" = @{ Lo = "0";           Hi = "0.5";        Why = "bogie pivot friction 0-0.5 N*m" }
    "arm_accel_scale"           = @{ Lo = "0.75";        Hi = "1.25";       Why = "arm acceleration +/-25 %" }
}
if ($Only) { foreach ($k in @($sweep.Keys)) { if ($Only -notcontains $k) { $sweep.Remove($k) } } }

if (-not $UnityPath) {
    $version = (Select-String -Path (Join-Path $project "ProjectSettings\ProjectVersion.txt") -Pattern "m_EditorVersion: (.+)").Matches[0].Groups[1].Value.Trim()
    $UnityPath = "C:\Program Files\Unity\Hub\Editor\$version\Editor\Unity.exe"
}

function Invoke-Run([string]$name, [string[]]$params) {
    $out = Join-Path $sweepDir $name
    New-Item -ItemType Directory -Force $out | Out-Null
    foreach ($f in "physics.json", "terrain.json") { Remove-Item (Join-Path $results "$Variant\$f") -ErrorAction SilentlyContinue }
    $unityArgs = @("-batchmode", "-nographics", "-projectPath", "`"$project`"", "-runTests", "-testPlatform", "PlayMode",
                   "-testFilter", "RoverPhysicsTests|TerrainCourseTests", "-testResults", "`"$(Join-Path $out 'tests.xml')`"",
                   "-variant", $Variant, "-logFile", "`"$(Join-Path $out 'tests.log')`"")
    foreach ($p in $params) { $unityArgs += @("-param", $p) }
    Write-Host "=== $name $($params -join ' ')" -ForegroundColor Cyan
    Start-Process -FilePath $UnityPath -ArgumentList $unityArgs -Wait -NoNewWindow
    foreach ($f in "physics.json", "terrain.json") { Copy-Item (Join-Path $results "$Variant\$f") $out -ErrorAction SilentlyContinue }
}

if (-not $ReportOnly) {
    if (-not (Test-Path $UnityPath)) { throw "Unity not found at $UnityPath (pass -UnityPath)." }
    if (Get-Process Unity -ErrorAction SilentlyContinue) { throw "Close the Unity editor first (batch mode cannot open a project that is open)." }
    Invoke-Run "baseline" @()
    foreach ($key in $sweep.Keys) {
        Invoke-Run "${key}_lo" @("$key=$($sweep[$key].Lo)")
        Invoke-Run "${key}_hi" @("$key=$($sweep[$key].Hi)")
    }
}

# ---- report --------------------------------------------------------------------------------------------
function Get-Metrics([string]$name) {
    $dir = Join-Path $sweepDir $name
    $p = if (Test-Path (Join-Path $dir "physics.json")) { Get-Content (Join-Path $dir "physics.json") -Raw | ConvertFrom-Json } else { $null }
    $t = if (Test-Path (Join-Path $dir "terrain.json")) { Get-Content (Join-Path $dir "terrain.json") -Raw | ConvertFrom-Json } else { $null }
    $lanes = if ($t) { @($t.lanes.PSObject.Properties) } else { @() }
    $armTimes = if ($p -and $p.arm) { @($p.arm.joints.PSObject.Properties.Value | Where-Object { $_.time_to_target_s } | ForEach-Object { [double]$_.time_to_target_s }) } else { @() }
    [ordered]@{
        AccelMps2  = if ($p.acceleration) { [double]$p.acceleration.measured_accel_mps2 } else { $null }
        DecelMps2  = if ($p.acceleration) { [double]$p.acceleration.measured_decel_mps2 } else { $null }
        StopPitch  = if ($p.acceleration) { [double]$p.acceleration.ramped_stop_pitch_change_deg } else { $null }
        SettleSec  = if ($p.settle -and $null -ne $p.settle.settle_time_s) { [double]$p.settle.settle_time_s } else { $null }
        SlipPct    = if ($p.drive) { [double]$p.drive.slip_percent } else { $null }
        DriftDeg   = if ($p.drive) { [double]$p.drive.heading_drift_deg } else { $null }
        ArmTimeSec = if ($armTimes.Count) { [math]::Round(($armTimes | Measure-Object -Average).Average, 3) } else { $null }
        Lanes      = if ($lanes.Count) { @($lanes | Where-Object { $_.Value.crossed }).Count } else { $null }
        Ramp20M    = if ($t) { [double]$t.lanes.ramp_20.progress_m } else { $null }
        Ramp30M    = if ($t) { [double]$t.lanes.ramp_30.progress_m } else { $null }
        Step15M    = if ($t) { [double]$t.lanes.step_15.progress_m } else { $null }
        # worst tilt on the lanes the rover normally crosses (ramp_30 is at its tipping limit and often flips)
        TiltDeg    = if ($lanes.Count) { ($lanes | Where-Object { $_.Name -ne "ramp_30" } | ForEach-Object { [double]$_.Value.max_tilt_deg } | Measure-Object -Maximum).Maximum } else { $null }
        DesertM    = if ($t -and $t.desert) { [double]$t.desert.progress_m } else { $null }
    }
}

# A "meaningful change" per metric: roughly what an algorithm test would notice. Each parameter's effect on a
# metric = |high - low| / scale; Impact = sum over metrics (each capped at 3). Absolute scales, because several
# baselines are near zero (drift, slip, stop pitch) and relative changes would exaggerate them.
$scale = [ordered]@{ AccelMps2 = 0.2; DecelMps2 = 0.2; StopPitch = 2; SettleSec = 0.5; SlipPct = 5; DriftDeg = 2;
                     ArmTimeSec = 0.1; Lanes = 1; Ramp20M = 2; Ramp30M = 2; Step15M = 2; TiltDeg = 3; DesertM = 5 }
$base = Get-Metrics "baseline"
$rows = @()
foreach ($key in $sweep.Keys) {
    $lo = Get-Metrics "${key}_lo"; $hi = Get-Metrics "${key}_hi"
    $row = [ordered]@{ Parameter = $key; Range = $sweep[$key].Why }
    $total = 0.0
    foreach ($m in $scale.Keys) {
        if ($null -eq $lo[$m] -or $null -eq $hi[$m]) { $row[$m] = ""; continue }
        $effect = [math]::Abs([double]$hi[$m] - [double]$lo[$m]) / $scale[$m]
        $row[$m] = "{0:N1}" -f $effect
        $total += [math]::Min($effect, 3.0)
    }
    $row["Impact"] = [math]::Round($total, 2)
    $rows += [pscustomobject]$row
}
$ranked = $rows | Sort-Object Impact -Descending
Write-Host "`n=== Sensitivity: |high - low| in units of a meaningful change (1.0 = noticeable; see `$scale) ===" -ForegroundColor Cyan
Write-Host ("Baseline: " + (($base.GetEnumerator() | ForEach-Object { "$($_.Key) $($_.Value)" }) -join "  "))
$ranked | Format-Table -AutoSize | Out-String -Width 400 | Write-Host
Write-Host "Measure in this order (highest impact first):"
$i = 1; foreach ($r in $ranked) { Write-Host ("  {0}. {1}  ({2}, impact {3})" -f $i, $r.Parameter, $r.Range, $r.Impact); $i++ }
$ranked | ConvertTo-Json -Depth 4 | Set-Content (Join-Path $sweepDir "ranking.json") -Encoding utf8
Write-Host "`nSaved: $(Join-Path $sweepDir 'ranking.json')"
