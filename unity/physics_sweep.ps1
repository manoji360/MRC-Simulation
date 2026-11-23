<#
.SYNOPSIS
  Compare Unity physics settings for the rover: accuracy, stability and speed.

.DESCRIPTION
  For each profile, edits ProjectSettings/DynamicsManager.asset (solver type, iterations, friction),
  runs the full PlayMode suite (physics + terrain tests) with that timestep, and keeps the results in
  FeasibilityResults/sweep/<profile>/. The original settings file is always restored at the end.
  Needs an imported robot (run run_feasibility.ps1 first) and Unity must be closed.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File unity\physics_sweep.ps1
  powershell -ExecutionPolicy Bypass -File unity\physics_sweep.ps1 -Profiles baseline,tgs_patch
#>
param(
    [string[]]$Profiles,
    [string]$Variant = "tuned",
    [string]$UnityPath
)
$ErrorActionPreference = "Stop"
if ($Profiles) { $Profiles = @($Profiles | ForEach-Object { $_ -split "," } | ForEach-Object { $_.Trim() } | Where-Object { $_ }) }

$project  = Join-Path $PSScriptRoot "MRC-Rover"
$settings = Join-Path $project "ProjectSettings\DynamicsManager.asset"
$results  = Join-Path $project "FeasibilityResults"
$sweepDir = Join-Path $results "sweep"

# solver: 0 = PGS (Projected Gauss-Seidel), 1 = TGS (Temporal Gauss-Seidel)
$all = [ordered]@{
    "baseline"   = @{ Solver = 0; Iter = 6;  Vel = 1; Patch = 0; Dt = "0.01";  Note = "PGS 6/1, 100 Hz (Unity default solver)" }
    "pgs_12_4"   = @{ Solver = 0; Iter = 12; Vel = 4; Patch = 0; Dt = "0.01";  Note = "PGS, more iterations" }
    "tgs"        = @{ Solver = 1; Iter = 6;  Vel = 1; Patch = 0; Dt = "0.01";  Note = "TGS solver (recommended for articulations)" }
    "tgs_patch"  = @{ Solver = 1; Iter = 6;  Vel = 1; Patch = 1; Dt = "0.01";  Note = "TGS + improved patch friction" }
    "tgs_50hz"   = @{ Solver = 1; Iter = 6;  Vel = 1; Patch = 1; Dt = "0.02";  Note = "TGS + patch, 50 Hz (cheaper)" }
    "tgs_200hz"  = @{ Solver = 1; Iter = 6;  Vel = 1; Patch = 1; Dt = "0.005"; Note = "TGS + patch, 200 Hz (accuracy reference)" }
    "pgs_200hz"  = @{ Solver = 0; Iter = 6;  Vel = 1; Patch = 0; Dt = "0.005"; Note = "PGS, 200 Hz (accuracy reference)" }
}
if (-not $Profiles) { $Profiles = @($all.Keys) }

if (-not $UnityPath) {
    $version = (Select-String -Path (Join-Path $project "ProjectSettings\ProjectVersion.txt") -Pattern "m_EditorVersion: (.+)").Matches[0].Groups[1].Value.Trim()
    $UnityPath = "C:\Program Files\Unity\Hub\Editor\$version\Editor\Unity.exe"
}
if (-not (Test-Path $UnityPath)) { throw "Unity not found at $UnityPath (pass -UnityPath)." }
if (Test-Path (Join-Path $project "Temp\UnityLockfile")) {
    $open = Get-Process Unity -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowTitle -like "MRC-Rover*" }
    if ($open) { throw "The MRC-Rover project is open in the Unity Editor. Close it and run again." }
}

function Set-Setting([string]$text, [string]$key, $value) {
    return [regex]::Replace($text, "(?m)^(\s*${key}:\s*).*$", "`${1}$value")
}

$original = Get-Content $settings -Raw
$rows = @()
try {
    foreach ($name in $Profiles) {
        if (-not $all.Contains($name)) { throw "Unknown profile '$name'. Known: $($all.Keys -join ', ')" }
        $p = $all[$name]
        Write-Host "`n=== $name : $($p.Note) ===" -ForegroundColor Cyan
        $text = $original
        $text = Set-Setting $text "m_SolverType" $p.Solver
        $text = Set-Setting $text "m_DefaultSolverIterations" $p.Iter
        $text = Set-Setting $text "m_DefaultSolverVelocityIterations" $p.Vel
        $text = Set-Setting $text "m_ImprovedPatchFriction" $p.Patch
        [IO.File]::WriteAllText($settings, $text)

        $out = Join-Path $sweepDir $name
        New-Item -ItemType Directory -Force $out | Out-Null
        foreach ($f in "physics.json", "terrain.json") { Remove-Item (Join-Path $results "$Variant\$f") -ErrorAction SilentlyContinue }
        $xml = Join-Path $out "tests.xml"
        $unityArgs = @("-batchmode", "-nographics", "-projectPath", "`"$project`"", "-runTests", "-testPlatform", "PlayMode",
                       "-testResults", "`"$xml`"", "-variant", $Variant, "-dt", $p.Dt, "-logFile", "`"$(Join-Path $out 'tests.log')`"")
        $timer = [Diagnostics.Stopwatch]::StartNew()
        Start-Process -FilePath $UnityPath -ArgumentList $unityArgs -Wait -NoNewWindow | Out-Null
        $timer.Stop()
        foreach ($f in "physics.json", "terrain.json") { Copy-Item (Join-Path $results "$Variant\$f") $out -ErrorAction SilentlyContinue }

        $phy = Get-Content (Join-Path $out "physics.json") -ErrorAction SilentlyContinue | ConvertFrom-Json
        $ter = Get-Content (Join-Path $out "terrain.json") -ErrorAction SilentlyContinue | ConvertFrom-Json
        $cases = @(); if (Test-Path $xml) { $cases = @(([xml](Get-Content $xml)).SelectNodes("//test-case")) }
        $lanes = if ($ter) { @($ter.lanes.PSObject.Properties) } else { @() }
        $rows += [pscustomobject]@{
            Profile    = $name
            Hz         = [math]::Round(1 / [double]$p.Dt)
            Tests      = "{0}/{1}" -f @($cases | Where-Object { $_.result -eq "Passed" }).Count, $cases.Count
            MsPerStep  = if ($phy) { $phy.performance.ms_per_step } else { "" }
            RealTimeX  = if ($phy) { $phy.performance.real_time_factor } else { "" }
            SettleSec  = if ($phy) { $phy.settle.settle_time_s } else { "" }
            DriveM     = if ($phy) { $phy.drive.distance_forward_m } else { "" }
            SlipPct    = if ($phy) { $phy.drive.slip_percent } else { "" }
            ArmErrDeg  = if ($phy) { $phy.arm.worst_error_deg } else { "" }
            Lanes      = "{0}/{1}" -f @($lanes | Where-Object { $_.Value.crossed }).Count, $lanes.Count
            Ramp30     = if ($ter) { if ($ter.lanes.ramp_30.crossed) { "crossed" } elseif ($ter.lanes.ramp_30.flipped) { "flipped" } else { "stuck" } } else { "" }
            DesertM    = if ($ter) { $ter.desert.progress_m } else { "" }
            WallSec    = [math]::Round($timer.Elapsed.TotalSeconds)
        }
    }
}
finally {
    [IO.File]::WriteAllText($settings, $original)
    Write-Host "`nRestored the original DynamicsManager.asset." -ForegroundColor DarkGray
}

Write-Host "`n=== Physics settings sweep ($Variant) ===" -ForegroundColor Cyan
$rows | Format-Table -AutoSize | Out-String -Width 250 | Write-Host
$rows | ConvertTo-Json | Set-Content (Join-Path $sweepDir "summary.json")
Write-Host "Per-profile details: $sweepDir\<profile>\physics.json and terrain.json"
