[CmdletBinding()]
param(
    [ValidateRange(1, 65535)]
    [int]$Port = 5600,

    [ValidateRange(0, 1000000)]
    [int]$MaxFrames = 0,

    [switch]$Headless,

    [string]$SaveAs
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$linuxRepo = "/home/james/Repos/house-bot"

if ([string]::IsNullOrWhiteSpace($SaveAs)) {
    $SaveAs = "c920-live-" + (Get-Date -Format "yyyyMMdd-HHmmss")
}

$outputDirectory = Join-Path $repoRoot "data/output"
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$stdoutPath = Join-Path $outputDirectory "$SaveAs.stdout.log"
$stderrPath = Join-Path $outputDirectory "$SaveAs.stderr.log"

$slamArgs = @(
    "-d", "Ubuntu",
    "--cd", $linuxRepo,
    "--",
    "./scripts/run_mast3r_slam.sh",
    "tcp-listen://0.0.0.0:$Port",
    "--config", "$linuxRepo/config/c920-live.yaml",
    "--save-as", $SaveAs
)
if ($MaxFrames -gt 0) {
    $slamArgs += @("--max-frames", $MaxFrames.ToString())
}
if ($Headless) {
    $slamArgs += "--no-viz"
}

Write-Host "Starting live SLAM listener on TCP port $Port"
$slamProcess = Start-Process -FilePath "wsl.exe" -WindowStyle Hidden `
    -ArgumentList $slamArgs `
    -RedirectStandardOutput $stdoutPath `
    -RedirectStandardError $stderrPath `
    -PassThru

Start-Sleep -Seconds 2

$streamScript = Join-Path $PSScriptRoot "stream_webcam.ps1"
$streamExit = 1
for ($attempt = 1; $attempt -le 10; $attempt++) {
    & $streamScript -Port $Port
    $streamExit = $LASTEXITCODE
    if ($streamExit -eq 0 -or $slamProcess.HasExited) {
        break
    }
    Write-Host "Stream connection attempt $attempt failed; retrying..."
    Start-Sleep -Seconds 1
}

if (-not $slamProcess.HasExited) {
    Wait-Process -Id $slamProcess.Id -Timeout 30 -ErrorAction SilentlyContinue
}
$slamProcess.Refresh()

Write-Host ""
Write-Host "SLAM stdout: $stdoutPath"
Write-Host "SLAM stderr: $stderrPath"
Write-Host "Saved run name: $SaveAs"

if ($slamProcess.HasExited) {
    exit $slamProcess.ExitCode
}

throw "The camera stream ended but the SLAM process is still running."
