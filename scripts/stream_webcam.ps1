[CmdletBinding()]
param(
    [string]$CameraName = "HD Pro Webcam C920",

    [string]$Destination,

    [ValidateRange(1, 65535)]
    [int]$Port = 5600,

    [ValidateRange(320, 1920)]
    [int]$Width = 960,

    [ValidateRange(240, 1080)]
    [int]$Height = 540,

    [ValidateRange(1, 60)]
    [int]$FrameRate = 30
)

$ErrorActionPreference = "Stop"
$ffmpeg = Get-Command ffmpeg -ErrorAction Stop

if ([string]::IsNullOrWhiteSpace($Destination)) {
    $addresses = (& wsl.exe -d Ubuntu -- hostname -I).Trim() -split "\s+"
    $Destination = $addresses[0]
}

$streamUrl = "tcp://${Destination}:$Port"
Write-Host "Streaming $CameraName to $streamUrl"
Write-Host "Requested output: ${Width}x${Height} at $FrameRate FPS"

$ffmpegArgs = @(
    "-hide_banner",
    "-loglevel", "warning",
    "-f", "dshow",
    "-rtbufsize", "256M",
    "-video_size", "1280x720",
    "-framerate", $FrameRate.ToString(),
    "-vcodec", "mjpeg",
    "-i", "video=$CameraName",
    "-an",
    "-vf", "scale=${Width}:${Height}",
    "-c:v", "libx264",
    "-preset", "ultrafast",
    "-tune", "zerolatency",
    "-g", "15",
    "-bf", "0",
    "-pix_fmt", "yuv420p",
    "-flush_packets", "1",
    "-muxdelay", "0",
    "-muxpreload", "0",
    "-f", "mpegts",
    $streamUrl
)

& $ffmpeg.Source @ffmpegArgs
exit $LASTEXITCODE
