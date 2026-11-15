<#
.SYNOPSIS
  Unity physics-backend feasibility test: URDF -> urdf_pipeline -> Unity import -> physics tests.

.DESCRIPTION
  For each variant:
    1. python -m urdf_pipeline prepare   (urdf-pipeline/output/<variant>)
    2. Unity batch import                (Assets/Robots/<variant>/, FeasibilityResults/<variant>/import.json)
    3. Unity PlayMode physics tests      (FeasibilityResults/<variant>/physics.json + tests.xml)
  then prints a summary table.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File unity\run_feasibility.ps1
  powershell -ExecutionPolicy Bypass -File unity\run_feasibility.ps1 -Variants collapsed -Urdf C:\robots\my_robot.urdf
#>
param(
    # convex-unity (Unity's own convex hulls instead of VHACD) is a comparison only: it loses
    # most colliders on the OSU rover, so it is not in the default list.
    [string[]]$Variants = @("default", "collapsed", "split-meshes"),
    [string]$Urdf,
    [string]$UnityPath,
    [string]$Python
)
$ErrorActionPreference = "Stop"

$repo    = Split-Path -Parent $PSScriptRoot
$project = Join-Path $PSScriptRoot "MRC-Rover"
$results = Join-Path $project "FeasibilityResults"
if (-not $Urdf) { $Urdf = Join-Path $repo "urdf-pipeline\input\rover.urdf" }

# Variant -> extra arguments for `prepare` and for the Unity import.
$variantSettings = @{
    "default"      = @{ Prepare = @();                              Convex = "vhacd" }
    "collapsed"    = @{ Prepare = @("--collapse-frames", "always"); Convex = "vhacd" }
    "split-meshes" = @{ Prepare = @("--split-meshes");              Convex = "vhacd" }
    "convex-unity" = @{ Prepare = @();                              Convex = "unity" }
}

function Find-Unity {
    $wanted = (Select-String -Path (Join-Path $project "ProjectSettings\ProjectVersion.txt") -Pattern "m_EditorVersion: (.+)").Matches[0].Groups[1].Value.Trim()
    $candidates = @(
        "C:\Program Files\Unity\Hub\Editor\$wanted\Editor\Unity.exe",
        "$env:ProgramFiles\Unity $wanted\Editor\Unity.exe"
    )
    foreach ($c in $candidates) { if (Test-Path $c) { return $c } }
    throw "Unity $wanted not found. Install it with Unity Hub, or pass -UnityPath 'C:\...\Unity.exe'."
}

if (-not $UnityPath) { $UnityPath = Find-Unity }
if (-not $Python) {
    $venv = Join-Path $repo ".venv\Scripts\python.exe"
    $Python = if (Test-Path $venv) { $venv } else { "python" }
}
if (-not (Test-Path $Urdf)) { throw "URDF not found: $Urdf (generate it with urdf-pipeline/env/generate_input.sh, or pass -Urdf)." }
$env:PYTHONPATH = Join-Path $repo "urdf-pipeline"

function Invoke-Unity([string[]]$UnityArgs, [string]$LogFile) {
    $quoted = $UnityArgs | ForEach-Object { if ($_ -match '\s') { "`"$_`"" } else { $_ } }
    $p = Start-Process -FilePath $UnityPath -ArgumentList (@("-batchmode", "-nographics", "-projectPath", "`"$project`"") + $quoted + @("-logFile", "`"$LogFile`"")) -Wait -PassThru -NoNewWindow
    return $p.ExitCode
}

$summary = @()
$terrainSummary = @()
foreach ($variant in $Variants) {
    if (-not $variantSettings.ContainsKey($variant)) { throw "Unknown variant '$variant'. Known: $($variantSettings.Keys -join ', ')" }
    $settings = $variantSettings[$variant]
    $prepared = Join-Path $repo "urdf-pipeline\output\$variant"
    $outDir = Join-Path $results $variant
    New-Item -ItemType Directory -Force $outDir | Out-Null
    Write-Host "`n=== $variant ===" -ForegroundColor Cyan

    Write-Host "1/3 prepare"
    & $Python -m urdf_pipeline prepare $Urdf -o $prepared @($settings.Prepare)
    if ($LASTEXITCODE -ne 0) { $summary += [pscustomobject]@{ Variant = $variant; Result = "prepare failed" }; continue }

    Write-Host "2/3 Unity import"
    $code = Invoke-Unity @("-executeMethod", "RoverImport.ImportFromCommandLine", "-prepared", $prepared, "-variant", $variant, "-convex", $settings.Convex) (Join-Path $outDir "import.log")
    if ($code -ne 0) { $summary += [pscustomobject]@{ Variant = $variant; Result = "import failed (see import.log)" }; continue }

    Write-Host "3/3 Unity physics tests"
    $xml = Join-Path $outDir "tests.xml"
    Invoke-Unity @("-runTests", "-testPlatform", "PlayMode", "-testResults", $xml, "-variant", $variant) (Join-Path $outDir "tests.log") | Out-Null

    $imp = Get-Content (Join-Path $outDir "import.json") | ConvertFrom-Json
    $phy = Get-Content (Join-Path $outDir "physics.json") -ErrorAction SilentlyContinue | ConvertFrom-Json
    $ter = Get-Content (Join-Path $outDir "terrain.json") -ErrorAction SilentlyContinue | ConvertFrom-Json
    if ($ter) {
        $lanes = @($ter.lanes.PSObject.Properties)
        $terrainSummary += [pscustomobject]@{
            Variant   = $variant
            Crossed   = "{0} of {1}" -f @($lanes | Where-Object { $_.Value.crossed }).Count, $lanes.Count
            Blocked   = (@($lanes | Where-Object { -not $_.Value.crossed } | ForEach-Object { "$($_.Name) ($(if ($_.Value.flipped) {'flipped'} elseif ($_.Value.stuck) {'stuck'} else {'timeout'}))" }) -join ", ")
            MaxTiltDeg = ($lanes | ForEach-Object { $_.Value.max_tilt_deg } | Measure-Object -Maximum).Maximum
            Desert    = if ($ter.desert) { "{0} m in {1} s, max tilt {2} deg{3}" -f $ter.desert.progress_m, $ter.desert.seconds, $ter.desert.max_tilt_deg, $(if ($ter.desert.stuck) {", STUCK"} elseif ($ter.desert.flipped) {", FLIPPED"} else {""}) } else { "" }
        }
    }
    $cases = @{}
    if (Test-Path $xml) { ([xml](Get-Content $xml)).SelectNodes("//test-case") | ForEach-Object { $cases[$_.name] = $_.result } }
    $failed = @($cases.GetEnumerator() | Where-Object { $_.Value -ne "Passed" } | ForEach-Object { $_.Key })
    $summary += [pscustomobject]@{
        Variant      = $variant
        Result       = if ($cases.Count -eq 0) { "tests did not run (see tests.log)" } elseif ($failed.Count -eq 0) { "ALL PASS" } else { "FAIL: " + ($failed -join ", ") }
        Bodies       = $imp.articulation_bodies
        ImportSec    = $imp.import_seconds
        MassKg       = if ($phy) { "{0} / {1}" -f $phy.mass.unity_total_kg, $phy.mass.unity_expected_kg } else { "" }
        SettleMps    = if ($phy) { $phy.settle.final_max_link_speed_mps } else { "" }
        DriveM       = if ($phy) { "{0} of {1}" -f $phy.drive.distance_forward_m, $phy.drive.expected_distance_m } else { "" }
        ArmErrDeg    = if ($phy) { $phy.arm.worst_error_deg } else { "" }
        MsPerStep    = if ($phy) { $phy.performance.ms_per_step } else { "" }
        RealTimeX    = if ($phy) { $phy.performance.real_time_factor } else { "" }
    }
}

Write-Host "`n=== Feasibility summary ===" -ForegroundColor Cyan
$summary | Format-Table -AutoSize | Out-String -Width 250 | Write-Host
if ($terrainSummary.Count -gt 0) {
    Write-Host "=== Terrain course and Utah desert (0.5 m/s) ===" -ForegroundColor Cyan
    $terrainSummary | Format-Table -AutoSize | Out-String -Width 250 | Write-Host
}
Write-Host "Details: $results\<variant>\physics.json, terrain.json and import.json"
Write-Host "Drive it yourself: open unity\MRC-Rover in Unity, open Assets/Scenes/RoverTestEnvironment.unity and press Play."
