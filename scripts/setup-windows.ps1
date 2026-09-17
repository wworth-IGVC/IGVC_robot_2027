<#
.SYNOPSIS
    Sets up the IGVC 2027 development environment on Windows, and checks that
    everything it needs is actually present before it starts.

.DESCRIPTION
    Written for teammates who have not used Docker before. It checks each
    prerequisite and explains what is wrong in plain language when something
    is missing.

    THIS SCRIPT ONLY CHECKS PREREQUISITES. It does not build anything and it
    does not start anything. When it passes, the next and only step is the
    bootstrap, which must be run from a WSL2 Ubuntu shell and not from
    PowerShell:

        bash scripts/gazebo/bootstrap.sh

    That is deliberate. Docker Desktop's own Linux VM has no display, so a
    container started from PowerShell can never open a Gazebo or RViz window,
    and the failure is silent. See docs/GAZEBO_QUICKSTART.md section 0.

    Run this from the repository root:

        .\scripts\setup-windows.ps1

    Useful switches:

        -WithPerception  Also check for the extra disk the 14 GB Humble
                         perception image needs. Not needed for the simulator.
        -WithZed         Also check for the disk the 28.6 GB ZED SDK image
                         needs. Only for ZED camera code, and a ZED camera
                         cannot be used from Docker on Windows anyway.
        -Weights         Download the YOLOPv2 model weights (about 150 MB).
                         The Gazebo simulator does not use them.

.NOTES
    Compatible with Windows PowerShell 5.1 (the version that ships with
    Windows 10), so it avoids newer syntax like ternary and null-coalescing.

    Changed 2026-09-17: this used to end by building igvc_humble_fused_drive,
    a 14 GB image the simulator does not need, and then told you to open it
    with 'docker compose run', which creates a new container every time. Both
    are gone. The disk threshold used to be a flat 30 GB, which was neither
    measured nor right for either path.
#>

[CmdletBinding()]
param(
    [switch]$CheckOnly,
    [switch]$WithPerception,
    [switch]$WithZed,
    [switch]$Weights
)

$ErrorActionPreference = "Stop"
$script:Problems = @()
$script:Warnings = @()

function Write-Step($msg)  { Write-Host ""; Write-Host "==> $msg" -ForegroundColor Cyan }
function Write-Ok($msg)    { Write-Host "    [ OK ] $msg" -ForegroundColor Green }
function Write-Warn2($msg) { Write-Host "    [WARN] $msg" -ForegroundColor Yellow; $script:Warnings += $msg }
function Write-Bad($msg)   { Write-Host "    [FAIL] $msg" -ForegroundColor Red;   $script:Problems += $msg }
function Write-Info($msg)  { Write-Host "           $msg" -ForegroundColor DarkGray }

Write-Host ""
Write-Host "  IGVC 2027 - Windows setup" -ForegroundColor White
Write-Host "  ---------------------------------------------------------------"

# -----------------------------------------------------------------------------
# 1. Windows version
#
# Docker Desktop needs Windows 10 22H2 (build 19045) or Windows 11. Earlier
# Windows 10 builds are out of support and Docker Desktop refuses to install.
# -----------------------------------------------------------------------------
Write-Step "Checking Windows version"
$osBuild = [int](Get-CimInstance Win32_OperatingSystem).BuildNumber
$osName  = (Get-CimInstance Win32_OperatingSystem).Caption
Write-Info "$osName (build $osBuild)"

# Three different thresholds matter and they are easy to conflate:
#   19041  WSL2 itself
#   19044  WSLg, i.e. whether a Linux GUI window can ever appear (21H2)
#   19045  what current Docker Desktop requires (22H2)
# One teammate is on Windows 10, so say plainly what to do at each level.
Write-Info "Thresholds: WSL2 needs 19041, WSLg needs 19044, Docker Desktop needs 19045"

if ($osBuild -ge 22000) {
    Write-Ok "Windows 11 - the verified path"
} elseif ($osBuild -ge 19045) {
    Write-Ok "Windows 10 22H2 (build $osBuild) - supported, above all three thresholds"
    Write-Info "On Windows 10 you must install WSL from the MICROSOFT STORE, not"
    Write-Info "the older 'Windows Subsystem for Linux' Windows feature. The"
    Write-Info "feature version has no WSLg and gives you no windows at all."
    Write-Info "    wsl --update"
    Write-Info "Then confirm there is a 'WSLg version' line in:"
    Write-Info "    wsl --version"
} elseif ($osBuild -ge 19044) {
    Write-Warn2 "Windows 10 build $osBuild has WSLg but is below Docker Desktop's 19045."
    Write-Info "WHAT TO DO: update to 22H2 via Settings > Windows Update. That is"
    Write-Info "one optional update away and it is the fix. Until you do, current"
    Write-Info "Docker Desktop will refuse to install."
} elseif ($osBuild -ge 19041) {
    Write-Bad "Windows 10 build $osBuild is too old for WSLg (needs 19044)."
    Write-Info "WHAT TO DO: update to 22H2 via Settings > Windows Update."
    Write-Info "Until then this machine can NEVER show a Gazebo or RViz window,"
    Write-Info "no matter what else is installed. There is no workaround short"
    Write-Info "of updating Windows."
    Write-Info "It can still do useful work headless: render_check.sh, the"
    Write-Info "smoke test and autonomy_check.sh all run with no display. Pair"
    Write-Info "with someone whose machine shows windows for anything visual."
} else {
    Write-Bad "Windows build $osBuild is too old for WSL2 at all (needs 19041+)."
    Write-Info "WHAT TO DO: update Windows before continuing. Nothing here works."
}

# -----------------------------------------------------------------------------
# 2. WSL2
# -----------------------------------------------------------------------------
Write-Step "Checking WSL2"
$wslOk = $false
try {
    $wslOut = (wsl --status) 2>&1 | Out-String
    if ($LASTEXITCODE -eq 0) {
        # wsl.exe emits UTF-16; strip nulls so -match works
        $clean = ($wslOut -replace "`0", "")
        Write-Ok "WSL is installed"
        if ($clean -match "2") { Write-Info "Default version appears to be 2" }
        $wslOk = $true
    } else {
        Write-Bad "WSL does not appear to be installed."
    }
} catch {
    Write-Bad "WSL does not appear to be installed."
}
if (-not $wslOk) {
    Write-Info "Install it from an Administrator PowerShell:"
    Write-Info "    wsl --install"
    Write-Info "Then reboot."
}

# -----------------------------------------------------------------------------
# 3. Docker
# -----------------------------------------------------------------------------
Write-Step "Checking Docker"
$dockerOk = $false
$dockerCmd = Get-Command docker -ErrorAction SilentlyContinue
if ($null -eq $dockerCmd) {
    Write-Bad "The 'docker' command was not found."
    Write-Info "Install Docker Desktop: https://docker.com/products/docker-desktop"
    Write-Info "During install, keep 'Use WSL 2 instead of Hyper-V' checked."
} else {
    $dv = (docker --version) 2>&1 | Out-String
    Write-Ok $dv.Trim()

    docker info *> $null
    if ($?) {
        Write-Ok "Docker daemon is running"
        $dockerOk = $true
    } else {
        Write-Bad "Docker is installed but the daemon is not responding."
        Write-Info "Start Docker Desktop and wait for the whale icon in the"
        Write-Info "system tray to stop animating, then run this script again."
    }
}

if ($dockerOk) {
    $composeOut = (docker compose version) 2>&1 | Out-String
    if ($?) { Write-Ok $composeOut.Trim() }
    else    { Write-Bad "'docker compose' is unavailable. Update Docker Desktop." }
}

# -----------------------------------------------------------------------------
# 4. GPU (optional)
# -----------------------------------------------------------------------------
Write-Step "Checking NVIDIA GPU support (optional)"
if ($dockerOk) {
    $gpuNames = @()
    try {
        $gpuNames = Get-CimInstance Win32_VideoController |
                    Where-Object { $_.Name -match "NVIDIA" } |
                    Select-Object -ExpandProperty Name
    } catch { }

    if ($gpuNames.Count -eq 0) {
        Write-Warn2 "No NVIDIA GPU detected. That is fine - everything still"
        Write-Info "builds and runs on CPU, just slower for the vision nodes."
    } else {
        Write-Info ("Found: " + ($gpuNames -join ", "))
        Write-Info "Testing GPU passthrough into a container (may pull ~350 MB)..."
        # Do NOT use $? here. In Windows PowerShell 5.1, redirecting a native
        # command's stderr wraps each line in an ErrorRecord and sets $? to
        # false even when the exe exited 0. Check the output and exit code.
        $smi = (docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi 2>&1) | Out-String
        # Match on "NVIDIA-SMI", not "Driver Version". Under WSL2 the header
        # reads "NVIDIA-SMI x  KMD Version: y  CUDA UMD Version: z" and the
        # string "Driver Version" never appears, which false-alarms on every
        # Windows machine.
        if ($LASTEXITCODE -eq 0 -and $smi -match "NVIDIA-SMI") {
            Write-Ok "GPU is visible inside containers"
        } else {
            Write-Warn2 "GPU passthrough is not working."
            Write-Info "Make sure your NVIDIA driver is current on WINDOWS."
            Write-Info "Do not install Linux NVIDIA drivers inside WSL."
            Write-Info "This is optional - CPU builds still work."
        }
    }
} else {
    Write-Warn2 "Skipped - Docker is not running."
}

# -----------------------------------------------------------------------------
# 5. Disk space
# -----------------------------------------------------------------------------
Write-Step "Checking disk space"

# Measured on this project 2026-09-17, not guessed. For the Gazebo-only path,
# which is what the simulator needs and all tonight's work needs:
#
#   igvc-gazebo-jazzy unpacked on disk   5.57 GB   (docker image ls)
#   the same image as a saved tar        1.20 GB   (docker save, 14.6 s)
#   the clone plus its nine submodules   0.60 GB   (du, including .git)
#   the colcon workspace in-container    0.003 GB  (build + install)
#   ------------------------------------------------------------------
#   offline path, load the tar           7.4 GB measured
#
# The threshold is set above that, not at it, because the WSL2 VM's own
# docker_data.vhdx grows and never shrinks, and one image version usually
# lingers. Building from source instead of loading a tar also leaves build
# cache behind: this machine is carrying 29.06 GB of it across four images,
# 21.86 GB of which is shared, so the share belonging to any one image is NOT
# cleanly attributable and the build figure below is a margin, not a
# measurement. That is stated rather than dressed up as precision.
$needed = 12                                  # offline load, or an image already present
if ($WithPerception) { $needed = $needed + 20 }   # igvc-humble-fused-drive is 14.1 GB
if ($WithZed)        { $needed = $needed + 40 }   # igvc-zed-humble is 28.6 GB

$drive = (Get-Location).Drive.Name
$free  = [math]::Round((Get-PSDrive $drive).Free / 1GB, 1)

Write-Info ("WANTS : {0} GB free on {1}:" -f $needed, $drive)
Write-Info ("FOUND : {0} GB free on {1}:" -f $free, $drive)
Write-Info "Building the image rather than loading a saved one wants about 25 GB,"
Write-Info "because of the build cache it leaves behind."

if ($free -lt $needed) {
    Write-Bad ("Not enough free space: wants {0} GB, found {1} GB." -f $needed, $free)
    Write-Info "Reclaim the cheapest space first, which is Docker's build cache:"
    Write-Info "    docker builder prune"
    Write-Info "Then unused images:"
    Write-Info "    docker image prune -a"
    Write-Info "Note: docker_data.vhdx never shrinks on its own, and Windows 11"
    Write-Info "Home has no Hyper-V, so Optimize-VHD is unavailable. Pruning"
    Write-Info "frees space inside the VM but the file on C: stays the same size."
} else {
    Write-Ok ("Sufficient free space: wants {0} GB, found {1} GB" -f $needed, $free)
}

# -----------------------------------------------------------------------------
# 6. Repository state
# -----------------------------------------------------------------------------
Write-Step "Checking the repository"
if (-not (Test-Path ".git")) {
    Write-Bad "This is not a git repository."
    Write-Info "Run the script from the root of your IGVC_robot_2027 clone."
} else {
    Write-Ok "Git repository found"

    $sm = (git submodule status) 2>&1 | Out-String
    $lines = $sm -split "`n" | Where-Object { $_.Trim() -ne "" }
    $missing = $lines | Where-Object { $_.TrimStart().StartsWith("-") }

    if ($missing.Count -gt 0) {
        Write-Bad "$($missing.Count) submodule(s) are empty."
        Write-Info "The build WILL fail without them. Fix with:"
        Write-Info "    git submodule update --init --recursive"
    } else {
        Write-Ok "All $($lines.Count) submodules are populated"
    }
}

# -----------------------------------------------------------------------------
# 7. Model weights
# -----------------------------------------------------------------------------
Write-Step "Checking YOLOPv2 model weights"
$weightsPath = "models\yolopv2.pt"
if (Test-Path $weightsPath) {
    $mb = (Get-Item $weightsPath).Length / 1MB
    Write-Ok ("{0} present ({1:N1} MB)" -f $weightsPath, $mb)
} elseif (-not $Weights) {
    Write-Info "Not present, and not downloaded. That is correct for the simulator."
    Write-Info "These weights are for lane_segmentation_node (YOLOPv2), which needs"
    Write-Info "torch and does not run in the Gazebo image. The Hough lane detector"
    Write-Info "(lane_detection_node) needs no weights and no torch, and does run."
    Write-Info "Pass -Weights if you are working on YOLOPv2."
} elseif ($CheckOnly) {
    Write-Warn2 "Missing, and -CheckOnly was given. Run without it to download."
} else {
    Write-Info "Downloading (about 150 MB, once per machine)..."
    try {
        New-Item -ItemType Directory -Force -Path "models" | Out-Null
        $url = "https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt"
        $ProgressPreference = "SilentlyContinue"
        Invoke-WebRequest -Uri $url -OutFile $weightsPath -UseBasicParsing
        Write-Ok "Downloaded $weightsPath"
    } catch {
        Write-Warn2 "Download failed: $($_.Exception.Message)"
        Write-Info "You can retry later, or on Git Bash run:"
        Write-Info "    ./src/igvc_lane_detection/scripts/fetch_yolopv2_weights.sh"
    }
}

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
Write-Host ""
Write-Host "  ---------------------------------------------------------------"
if ($script:Problems.Count -gt 0) {
    Write-Host "  $($script:Problems.Count) problem(s) must be fixed:" -ForegroundColor Red
    foreach ($p in $script:Problems) { Write-Host "    - $p" -ForegroundColor Red }
    Write-Host ""
    Write-Host "  Fix these, then run the script again." -ForegroundColor Red
    exit 1
}
if ($script:Warnings.Count -gt 0) {
    Write-Host "  $($script:Warnings.Count) warning(s) - not blocking:" -ForegroundColor Yellow
    foreach ($w in $script:Warnings) { Write-Host "    - $w" -ForegroundColor Yellow }
}
Write-Host "  All required checks passed." -ForegroundColor Green

# -----------------------------------------------------------------------------
# Hand over. This script builds nothing, on purpose.
#
# It used to end by building igvc_humble_fused_drive, a 14.1 GB image the
# simulator does not use, and then told you to open it with
# 'docker compose run --rm', which creates a NEW container with a random name
# every time. That is how six gz sim processes once ran at once, and how a
# 14 GB download once happened over campus wifi for no reason.
#
# Everything from here runs from a WSL2 Ubuntu shell, because Docker
# Desktop's Linux VM has no display and a container started from PowerShell
# can never open a window. The failure is silent: the command appears to work
# and no window ever arrives.
# -----------------------------------------------------------------------------
Write-Host ""
Write-Host "  ---------------------------------------------------------------"
Write-Host "  Prerequisites are in place. Nothing was built." -ForegroundColor Green
Write-Host ""
Write-Host "  NEXT STEP, and it is not in PowerShell:" -ForegroundColor White
Write-Host ""
Write-Host "    1. Open a WSL2 Ubuntu shell   (type 'wsl', or launch Ubuntu)"
Write-Host "    2. cd to this repo under /mnt/c/..."
Write-Host "    3. bash scripts/gazebo/bootstrap.sh"
Write-Host ""
Write-Host "  That one command updates the submodules, gets the image, builds" -ForegroundColor DarkGray
Write-Host "  the workspace, and runs the GPU check and the smoke test," -ForegroundColor DarkGray
Write-Host "  printing PASS or FAIL for each step." -ForegroundColor DarkGray
Write-Host ""
Write-Host "  If someone handed you the image on a USB drive, use it and skip" -ForegroundColor White
Write-Host "  the download entirely (D: is /mnt/d from WSL2):" -ForegroundColor White
Write-Host ""
Write-Host "    IMAGE_TAR=/mnt/d/igvc-gazebo-jazzy.tar bash scripts/gazebo/bootstrap.sh"
Write-Host ""
Write-Host "  Full instructions: docs/GAZEBO_QUICKSTART.md" -ForegroundColor DarkGray
Write-Host ""

if ($CheckOnly) {
    Write-Host "  -CheckOnly was given. It makes no difference: this script" -ForegroundColor DarkGray
    Write-Host "  never changes anything except downloading weights with" -ForegroundColor DarkGray
    Write-Host "  -Weights." -ForegroundColor DarkGray
    Write-Host ""
}
exit 0
