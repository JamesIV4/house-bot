[CmdletBinding()]
param(
    [ValidateRange(1, 3600)]
    [int]$DurationSeconds = 60,

    [string]$CameraName = "HD Pro Webcam C920",

    [ValidateRange(320, 3840)]
    [int]$Width = 1280,

    [ValidateRange(240, 2160)]
    [int]$Height = 720,

    [ValidateRange(1, 60)]
    [int]$FrameRate = 30,

    [string]$OutputPath
)

$ErrorActionPreference = "Stop"

$ffmpeg = Get-Command ffmpeg -ErrorAction Stop
$ffprobe = Get-Command ffprobe -ErrorAction Stop
$repoRoot = Split-Path -Parent $PSScriptRoot

if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
    $OutputPath = Join-Path $repoRoot "data/input/c920-room-loop-$timestamp.mp4"
}

$outputDirectory = Split-Path -Parent $OutputPath
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null

Write-Host "Recording $CameraName at ${Width}x${Height}, requested ${FrameRate} FPS"
Write-Host "Move smoothly, avoid blank walls, and finish where the recording began."
Write-Host "Output: $OutputPath"

$ffmpegArgs = @(
    "-hide_banner",
    "-y",
    "-f", "dshow",
    "-rtbufsize", "256M",
    "-video_size", "${Width}x${Height}",
    "-framerate", $FrameRate.ToString(),
    "-vcodec", "mjpeg",
    "-i", "video=$CameraName",
    "-t", $DurationSeconds.ToString(),
    "-an",
    "-c:v", "libx264",
    "-preset", "ultrafast",
    "-crf", "18",
    "-pix_fmt", "yuv420p",
    "-movflags", "+faststart",
    $OutputPath
)

& $ffmpeg.Source @ffmpegArgs
if ($LASTEXITCODE -ne 0) {
    throw "FFmpeg capture failed with exit code $LASTEXITCODE"
}

Write-Host ""
Write-Host "Capture details:"
& $ffprobe.Source -v error -count_frames `
    -show_entries "stream=codec_name,width,height,avg_frame_rate,nb_read_frames" `
    -show_entries "format=duration,size" `
    -of "default=noprint_wrappers=1" $OutputPath

$wslPrefix = "\\wsl.localhost\Ubuntu\"
$resolvedOutput = [System.IO.Path]::GetFullPath($OutputPath)
if ($resolvedOutput.StartsWith($wslPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    $wslPath = "/" + $resolvedOutput.Substring($wslPrefix.Length).Replace("\", "/")
    Write-Host ""
    Write-Host "Run SLAM from WSL:"
    Write-Host "./scripts/run_mast3r_slam.sh '$wslPath' --no-viz --save-as c920-room-loop"
}
