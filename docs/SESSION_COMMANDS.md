# Command and link sheet

**Every command and link from the software session, in the order they come up,
with no explanation.** The explanations live in `GAZEBO_QUICKSTART.md` and
`GAZEBO_SETUP.md`. This sheet exists so nobody has to type anything.

**PS** means a PowerShell window. **PS-Admin** means PowerShell started with
Run as administrator. **WSL** means a WSL2 Ubuntu shell, which you open by
typing `wsl` in a terminal or launching Ubuntu from the Start menu.

**Getting this wrong is the most common failure of the evening: every Docker
command marked WSL must be run in a WSL2 shell, or no window will ever
appear.**

---

## The three whiteboard lines

WSL:

```bash
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
```

WSL:

```bash
docker pull ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
```

PS:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\setup-windows.ps1
```

---

## 1. Triage

PS, Administrator rights:

```powershell
([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
```

PS, Windows build number:

```powershell
winver
```

PS, graphics card:

```powershell
Get-CimInstance Win32_VideoController | Select-Object Name, DriverVersion
```

---

## 2. Install Git

PS-Admin:

```powershell
winget install --id Git.Git --exact --source winget --accept-source-agreements
```

Then open a NEW PowerShell window. PS:

```powershell
git --version
git config --global core.autocrlf input
```

Manual download if winget is unavailable:

```text
https://gitforwindows.org/
```

---

## 3. Install WSL2 and Ubuntu

PS-Admin:

```powershell
wsl --install
```

PS-Admin, reboot:

```powershell
Restart-Computer
```

After the reboot, PS:

```powershell
wsl --update
```

PS, checks:

```powershell
wsl --version
wsl -l -v
```

PS, only if `wsl -l -v` says VERSION 1:

```powershell
wsl --set-version Ubuntu 2
```

Manual instructions if `wsl --install` fails:

```text
https://learn.microsoft.com/windows/wsl/install-manual
```

---

## 4. Install Docker Desktop

PS-Admin:

```powershell
winget install --id Docker.DockerDesktop --exact --source winget
```

PS-Admin, reboot:

```powershell
Restart-Computer
```

Manual download:

```text
https://www.docker.com/products/docker-desktop
```

Then, in Docker Desktop: Settings, Resources, WSL Integration, enable your
distro, Apply and Restart.

PS, check:

```powershell
docker run hello-world
```

WSL, check again, this one is the one that matters:

```bash
docker run hello-world
```

---

## 5. Get the code

WSL:

```bash
cd /mnt/c
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
```

WSL, checks. First must print 9, second must print nothing:

```bash
git submodule status | wc -l
git submodule status | grep '^-'
```

WSL, only if a submodule is empty:

```bash
git submodule update --init --recursive
```

---

## 6. Check the machine

PS, from inside the repo folder:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\setup-windows.ps1
```

---

## 7. Set your domain ID, then run the one command

WSL, with your number from the whiteboard:

```bash
export ROS_DOMAIN_ID=3
```

WSL:

```bash
bash scripts/gazebo/bootstrap.sh
```

WSL, to make the domain ID stick in new windows:

```bash
echo 'export ROS_DOMAIN_ID=3' >> ~/.bashrc
```

WSL, to change the domain ID after the container already exists:

```bash
docker compose -f docker-compose.windows.yml down
export ROS_DOMAIN_ID=3
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
docker exec igvc_gazebo printenv ROS_DOMAIN_ID
```

---

## 8. Run the simulator

WSL, start the container:

```bash
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
```

WSL, the GPU and tier check:

```bash
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
```

WSL, open the world, you drive:

```bash
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

WSL, the robot drives itself:

```bash
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

WSL, **tier C only**, no Gazebo window:

```bash
docker exec -it igvc_gazebo bash -c "CAMERAS=front HEADLESS=1 RVIZ=1 NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

---

## 9. Drive it with the keyboard

WSL, a SECOND shell into the same container:

```bash
docker exec -it igvc_gazebo bash
```

Then inside that container shell:

```bash
source /root/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i` forward, `,` back, `j` and `l` turn, `k` stop.

---

## 10. The three checks

WSL, graphics card and tier:

```bash
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
```

WSL, the main test, wants `18 failed: 0`:

```bash
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh"
```

WSL, does it drive the course by itself:

```bash
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh"
```

---

## 11. Fixes

WSL, reset a stuck or doubled simulator:

```bash
docker compose -f docker-compose.windows.yml restart igvc_gazebo
```

WSL, stop everything:

```bash
docker compose -f docker-compose.windows.yml down
```

WSL, Windows line endings broke the scripts:

```bash
git config --global core.autocrlf input
git rm --cached -r . && git reset --hard
```

WSL, empty submodules:

```bash
git submodule update --init --recursive
```

WSL, navigation lost its startup race:

```bash
ros2 launch igvc_test_bringup gazebo_nav_test.launch.py nav2_delay:=25.0
```

WSL, look for leftover simulators:

```bash
docker exec igvc_gazebo ps -ef | grep "gz sim"
```

WSL, free disk space:

```bash
docker builder prune
docker image prune -a
```

WSL, why did the container die:

```bash
docker logs igvc_gazebo
```

WSL, check the WSL graphics libraries reached the container:

```bash
docker exec igvc_gazebo ls /usr/lib/wsl/lib
```

WSL, build the image locally when the registry is unreachable:

```bash
BUILD_IMAGE=1 bash scripts/gazebo/bootstrap.sh
```

---

## 12. Is the image still public

PS:

```powershell
docker logout ghcr.io
docker manifest inspect ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
```

JSON back means public. `denied` means private.

---

## 13. Links

The repository:

```text
https://github.com/wworth-IGVC/IGVC_robot_2027
```

The image, public package page:

```text
https://github.com/users/wworth-IGVC/packages/container/package/igvc-gazebo-jazzy
```

The image, visibility settings, owner only:

```text
https://github.com/users/wworth-IGVC/packages/container/igvc-gazebo-jazzy/settings
```

Git for Windows:

```text
https://gitforwindows.org/
```

Docker Desktop:

```text
https://www.docker.com/products/docker-desktop
```

WSL manual install:

```text
https://learn.microsoft.com/windows/wsl/install-manual
```

The full setup guide, in the repo:

```text
docs/GAZEBO_QUICKSTART.md
```

The reference, why everything is the way it is:

```text
docs/GAZEBO_SETUP.md
```

What is done and what is left:

```text
docs/GAZEBO_TODO.md
```

---

## 14. Image and topic names, for copying

The image:

```text
ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
ghcr.io/wworth-igvc/igvc-gazebo-jazzy:2026-09-17
```

The container, and the repo path inside it:

```text
igvc_gazebo
/root/ros2_ws/src/IGVC_robot_2026
```

The topics worth knowing:

```text
/cmd_vel
/odom
/scan
/front_zed_camera_x/zed_node/rgb/color/rect/image
/lane_ground_truth
/lane_map
/lane_costmap
```
