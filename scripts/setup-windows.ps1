<#
.SYNOPSIS
    Sets up the IGVC 2027 development environment on Windows, and checks that
    everything it needs is actually present before it starts.

.DESCRIPTION
    Written for teammates who have not used Docker before. It checks each
    prerequisite, explains what is wrong in plain language when something is
    missing, and only then builds the container image.

    Run it from the repository root:

        .\scripts\setup-windows.ps1

    Useful switches:

        -CheckOnly     Run the checks and stop. Changes nothing.
        -SkipWeights   Do not download the YOLOPv2 model weights (~200 MB).
        -WithZed       Also build the large ZED SDK image (15-20 GB).
                       Only needed if you are working on ZED camera code.
                       A ZED camera cannot be used from Docker on Windows.

.NOTES
    Compatible with Windows PowerShell 5.1 (the version that ships with
    Windows 10), so it avoids newer syntax like ternary and null-coalescing.
#>

[CmdletBinding()]
param(
    [switch]$CheckOnly,
    [switch]$SkipWeights,
    [switch]$WithZed
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

if ($osBuild -ge 22000) {
    Write-Ok "Windows 11 - supported"
} elseif ($osBuild -ge 19045) {
    Write-Ok "Windows 10 22H2 - supported"
} elseif ($osBuild -ge 19041) {
    Write-Warn2 "Windows 10 build $osBuild is older than 22H2 (19045)."
    Write-Info "WSL2 works, but current Docker Desktop requires 22H2 or newer."
    Write-Info "Update via Settings > Windows Update, or install an older"
    Write-Info "Docker Desktop release. Try the build and see if it works."
} else {
    Write-Bad "Windows build $osBuild is too old for WSL2 (needs 19041+)."
    Write-Info "Update Windows before continuing."
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
$drive = (Get-Location).Drive.Name
$free  = (Get-PSDrive $drive).Free / 1GB
$needed = 30
if ($WithZed) { $needed = 60 }
Write-Info ("Free on {0}: {1:N1} GB (need about {2} GB)" -f $drive, $free, $needed)
if ($free -lt $needed) {
    Write-Bad ("Not enough free space. Need roughly {0} GB." -f $needed)
    Write-Info "Reclaim Docker space with: docker system prune -a"
} else {
    Write-Ok "Sufficient free space"
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
$weights = "models\yolopv2.pt"
if (Test-Path $weights) {
    $mb = (Get-Item $weights).Length / 1MB
    Write-Ok ("{0} present ({1:N1} MB)" -f $weights, $mb)
} elseif ($SkipWeights) {
    Write-Warn2 "Missing, and -SkipWeights was given. Lane detection will not run."
} elseif ($CheckOnly) {
    Write-Warn2 "Missing. Run without -CheckOnly to download."
} else {
    Write-Info "Downloading (~200 MB, once per machine)..."
    try {
        New-Item -ItemType Directory -Force -Path "models" | Out-Null
        $url = "https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt"
        $ProgressPreference = "SilentlyContinue"
        Invoke-WebRequest -Uri $url -OutFile $weights -UseBasicParsing
        Write-Ok "Downloaded $weights"
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

if ($CheckOnly) {
    Write-Host ""
    Write-Host "  -CheckOnly was given, so nothing was built." -ForegroundColor DarkGray
    exit 0
}

# -----------------------------------------------------------------------------
# Build
# -----------------------------------------------------------------------------
Write-Step "Building the development image"
Write-Info "First build downloads about 4 GB and takes 10-20 minutes."
Write-Info "Later builds reuse the cache and are much faster."
Write-Host ""

docker compose -f docker-compose.windows.yml build igvc_humble_fused_drive
if (-not $?) {
    Write-Host ""
    Write-Bad "Image build failed. Scroll up for the first error."
    exit 1
}
Write-Ok "igvc-humble-fused-drive:latest built"

if ($WithZed) {
    Write-Step "Building the ZED SDK image (large)"
    Write-Info "This downloads a 1.6 GB SDK installer on top of a CUDA base."
    Write-Info "Expect 30+ minutes and 15-20 GB of disk."
    docker compose -f docker-compose.windows.yml build igvc_zed_humble
    if ($?) { Write-Ok "igvc-zed-humble:latest built" }
    else    { Write-Warn2 "ZED image build failed. The main image above still works." }
}

Write-Host ""
Write-Host "  ---------------------------------------------------------------"
Write-Host "  Setup complete." -ForegroundColor Green
Write-Host ""
Write-Host "  Open a shell inside the container:" -ForegroundColor White
Write-Host "    docker compose -f docker-compose.windows.yml run --rm igvc_humble_fused_drive"
Write-Host ""
Write-Host "  Then, inside the container, compile the workspace:" -ForegroundColor White
Write-Host "    source /opt/ros/humble/setup.bash"
Write-Host "    colcon build --symlink-install --base-paths /root/ros2_ws/src/IGVC_robot_2026/src"
Write-Host "    source /root/ros2_ws/install/setup.bash"
Write-Host ""
Write-Host "  Expect: Summary: 22 packages finished" -ForegroundColor DarkGray
Write-Host ""
