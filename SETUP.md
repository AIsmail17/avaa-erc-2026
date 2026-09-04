# ERC 2026 — Setup Guide for Team AVAA

**Machine:** i7-12850HX · 64 GB RAM · NVIDIA RTX A4500 Laptop (driver 596.86) · Windows 11 Pro 26200
**Written:** 2026-08-25 · **Phase 1 deadline:** 2026-09-15

This guide assumes you have never used WSL, Docker, or ROS 2. Every step says what it does,
what to type, and how to know it worked. Follow it in order — later steps depend on earlier ones.

---

## Before you start: the mental model

You are building a stack of four layers. Each one sits inside the one above it.

```
Windows 11                 ← your normal desktop
  └── WSL2 / Ubuntu 24.04  ← a real Linux system running inside Windows
        └── Docker         ← runs sealed "containers" (pre-built Linux environments)
              └── ERC container  ← Ubuntu 22.04 + ROS 2 Humble + Gazebo + the TIAGo Pro robot
```

**Why so many layers?** The competition ships one official Docker image. The judges run *your*
code inside *that exact image*. If it works there, it works for them. Docker needs Linux, and
WSL2 gives you Linux inside Windows without dual-booting.

### Vocabulary you'll see constantly

| Term | What it actually means |
|---|---|
| **WSL2** | Windows Subsystem for Linux — a full Ubuntu running alongside Windows |
| **Docker image** | A frozen snapshot of a whole Linux environment. Read-only, downloaded once. |
| **Docker container** | A running copy of an image. You work inside it. |
| **ROS 2** | The robotics framework. Programs are "nodes" that talk over named channels. |
| **Topic** | A named channel, e.g. `/cmd_vel`. Nodes publish to it or subscribe to it. |
| **Node** | One running program in ROS 2 (e.g. your camera detector). |
| **Launch file** | A Python script that starts several nodes at once. Yours is `solution.launch.py`. |
| **Gazebo** | The 3D physics simulator. This is where the virtual library and robot live. |
| **RViz 2** | A viewer for what the robot *thinks* — camera feeds, laser scans, planned paths. |
| **colcon** | The ROS 2 build tool. `colcon build` compiles your packages. |
| **Shell / terminal** | The black text window where you type commands. |

> **Reading commands:** lines starting with `#` are comments, don't type them. When this guide
> says "in PowerShell", use a Windows PowerShell window. When it says "in Ubuntu", use the
> Ubuntu terminal. Mixing these up is the single most common beginner mistake.

---

## Where everything lives

Everything stays on your D: drive, as intended.

| What | Where |
|---|---|
| PDFs, notes, this guide | `D:\Projects\Emirates Robotics Competition\` |
| Report drafts (deliverable D3) | `D:\Projects\Emirates Robotics Competition\report\` |
| Screen recordings (deliverable D2) | `D:\Projects\Emirates Robotics Competition\video\` |
| Datasheets, saved web references | `D:\Projects\Emirates Robotics Competition\reference\` |
| **The entire Ubuntu system + all code** | `D:\Projects\Emirates Robotics Competition\wsl\` |

That last row is the important one. WSL stores all of Linux inside a single large file called
`ext4.vhdx` (a virtual hard disk). Step 3 moves that file onto D:, into the competition folder.
So when you write code at `~/erc/` inside Ubuntu, those bytes physically live on your D: drive,
inside this project folder — even though the path looks like Linux.

You can browse those files from Windows Explorer at `\\wsl.localhost\Ubuntu-24.04\home\YOUR_NAME\erc\`.

> **Why not just put the code in `D:\Projects\...\code\` directly?** Because Docker would have to
> read it across the Windows↔Linux boundary, which is roughly 10–20× slower. A build that takes
> 40 seconds becomes 10 minutes. Keep code inside Linux; the VHDX already puts it on D:.

> **Never back up or sync the `wsl\` folder** with OneDrive/Dropbox — it's a live multi-gigabyte
> disk image. Your *code* gets backed up by pushing to GitHub instead.

---

# PHASE A — Windows setup

## Step 0 — Preflight (already verified ✅)

These were checked on 2026-08-25 and all passed. No action needed:

- Hardware virtualization: **enabled**
- NVIDIA driver: **596.86**, RTX A4500 detected
- Free space on D:: **790 GB** (you need ~30 GB)
- Architecture: **x86_64** (required — ARM is not supported)

> **Do not install a Linux NVIDIA driver inside Ubuntu later.** The Windows driver already
> provides GPU access to WSL. Installing the Linux one on top breaks it. This trips up a lot
> of people following generic ROS tutorials.

---

## Step 1 — Install WSL2 and Ubuntu ✅ (done 2026-08-25)

**What this did:** Turned on the Windows features that host Linux, then installed Ubuntu 24.04.

Run in PowerShell **as Administrator** (`Win` → type `PowerShell` → right-click → Run as
administrator), then reboot when prompted:

```powershell
wsl --install -d Ubuntu-24.04
```

> **What actually happened here:** the first run installed the WSL engine (v2.7.12) and enabled
> the Windows features, but the Ubuntu download did **not** survive the reboot — `wsl -l -v`
> reported "no installed distributions". This is a common hiccup with the combined install.
>
> The fix was to install the distribution as a second, separate step — and because WSL 2.7
> supports `--location`, we pointed it straight at D: instead of installing to C: and moving
> it afterwards:
>
> ```powershell
> wsl --install -d Ubuntu-24.04 --location "D:\Projects\Emirates Robotics Competition\wsl\Ubuntu-24.04" --no-launch
> ```
>
> `--no-launch` skips the automatic first-run so the account can be created deliberately.
> **This made Step 3 unnecessary** — the disk was never on C: to begin with.

### Step 1b — Create your Linux user account

Press `Win`, type `Ubuntu`, and open it (or run `wsl` from any terminal).

It will ask you to create a UNIX username and password:

- **Username:** lowercase, no spaces. Something short like `avaa` or your first name.
  You'll type this a lot.
- **Password:** you will need this for every `sudo` command. **Nothing appears on screen as you
  type it** — no dots, no asterisks. That's normal, not a frozen terminal. Type it and press Enter.
- Write both down somewhere. This is completely separate from your Windows login.

**Verify** — back in PowerShell (normal, non-admin is fine):

```powershell
wsl -l -v
```

You want to see `Ubuntu-24.04` with `VERSION` = **2**. If it says 1, run
`wsl --set-version Ubuntu-24.04 2`.

> **If `wsl -l -v` says "no installed distributions"** but `wsl --version` works, the engine
> installed but the distro didn't. Re-run the `--location` command in the Step 1 box above.

---

## Step 2 — Update WSL itself ✅ (already current)

**What this does:** WSL's own engine updates separately from Windows.

```powershell
wsl --update
```

Verified on 2026-08-25: **WSL 2.7.12.0**, kernel 6.18.33.2-2, WSLg 1.0.73.2. Well past the
2.2.1 minimum, and new enough to support `--location` at install time.

---

## Step 3 — Move Linux onto the D: drive ⏭️ (skipped — not needed)

Originally this step relocated `ext4.vhdx` from C: to D: after the fact. Step 1 installed it
straight to D: using `--location`, so there is nothing to move.

**Confirmed location:**

```
D:\Projects\Emirates Robotics Competition\wsl\Ubuntu-24.04\ext4.vhdx
```

Everything you do inside Ubuntu writes into that file, on D:. Nothing WSL-related was left
in `%LOCALAPPDATA%` on C:.

To re-verify at any time:

```powershell
Get-ChildItem "D:\Projects\Emirates Robotics Competition\wsl\Ubuntu-24.04"
```

<details>
<summary>For teammates setting up a second machine</summary>

Install straight to the target drive in one shot and skip the move entirely:

```powershell
wsl --install -d Ubuntu-24.04 --location "D:\path\you\want\Ubuntu-24.04" --no-launch
```

If their WSL is older than 2.4 and lacks `--location`, install normally then move it:

```powershell
wsl --shutdown
wsl --manage Ubuntu-24.04 --move "D:\path\you\want\Ubuntu-24.04"
```
</details>

---

# PHASE B — Inside Ubuntu

From here on, unless a block says "in PowerShell", type these in the **Ubuntu** terminal
(`Win` → type `Ubuntu` → Enter).

## Step 4 — Turn on systemd

**What this does:** systemd is Linux's service manager. Without it you'd have to start Docker by
hand every single session. With it, Docker starts automatically.

Copy this whole block — all four lines — and paste it in one go:

```bash
sudo tee /etc/wsl.conf > /dev/null <<'WSLCONF'
[boot]
systemd=true
WSLCONF
```

It will ask for the UNIX password you created in Step 1. (Remember: no characters appear as you type.)

Now restart Linux. **In PowerShell:**

```powershell
wsl --shutdown
```

Reopen Ubuntu and check:

```bash
systemctl is-system-running
```

`running` or `degraded` both mean success. `offline` means the file didn't take — recheck Step 4.

---

## Step 5 — Install base tools

**What this does:** Updates Ubuntu's package list, upgrades what's installed, then adds the tools
later steps need. `apt` is Ubuntu's app installer; `sudo` means "run as administrator".

```bash
sudo apt update && sudo apt upgrade -y
```

```bash
sudo apt install -y git curl wget ca-certificates gnupg build-essential x11-apps mesa-utils
```

The first one can take several minutes on a fresh system. Walls of scrolling text are normal.

---

## Step 6 — Make graphics use the RTX A4500 ⚠️ (needed a fix)

**What this does:** Gazebo is a 3D simulator — it needs real GPU rendering. This confirms Linux
can see your RTX A4500 before you waste an hour discovering otherwise.

```bash
glxinfo -B | grep -i "renderer\|device\|accelerated"
```

You want a line mentioning **RTX A4500** through a "D3D12" layer, and `Accelerated: yes`.
That D3D12 wrapper is normal and correct in WSL — it's how Linux OpenGL reaches a Windows GPU.

### What went wrong here, and the fix

On first check this machine reported:

```
Device: llvmpipe (LLVM 20.1.2, 256 bits)
Accelerated: no
```

`llvmpipe` means **software rendering** — the CPU drawing 3D by hand, a few frames per second.
Gazebo would have been unusable.

Everything needed was actually present (`/dev/dxg`, `/usr/lib/wsl/lib/libd3d12.so`,
`d3d12_dri.so`, and the `ld.wsl.conf` search path). Mesa simply wasn't selecting the D3D12
driver, and when forced to, it picked the **Intel UHD** integrated chip instead of the A4500 —
the hybrid-graphics trap this laptop is prone to.

Two environment variables fix it:

| Variable | Effect |
|---|---|
| `GALLIUM_DRIVER=d3d12` | Switches Mesa off llvmpipe onto hardware D3D12 |
| `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` | Picks the A4500 rather than Intel UHD |

**Both are needed.** With only the first, you get accelerated rendering on the wrong GPU.

They were made permanent in two places (so they apply to login shells *and* non-login/service
contexts):

```bash
sudo tee /etc/profile.d/99-wsl-gpu.sh > /dev/null <<'GPUENV'
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
GPUENV
```

```bash
printf 'GALLIUM_DRIVER=d3d12\nMESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA\n' | sudo tee -a /etc/environment
```

**Result after the fix:**

```
Device: D3D12 (NVIDIA RTX A4500 Laptop GPU)
Accelerated: yes
```

Benchmark: `glxgears` held **~110 FPS**. Hardware rendering confirmed.

> **If Gazebo is still slow inside the container**, these variables may not have propagated in.
> Check with `echo $GALLIUM_DRIVER` inside the container; if empty, add them to the
> `environment:` block of `docker/docker-compose.yml`.

### Visual test

```bash
xeyes
```

A little window with a pair of eyes that follow your mouse should appear on your Windows
desktop. Close it with `Ctrl+C`.

---

# PHASE C — Docker and GPU access

## Step 7 — Install Docker Engine

**What this does:** Installs Docker. Note: **not Docker Desktop**. The competition's scripts
expect Linux-native Docker so they can share your display and GPU with the container. Docker
Desktop routes through a different mechanism and complicates both.

These three blocks add Docker's official software source to Ubuntu. Run them one at a time.

```bash
sudo install -m 0755 -d /etc/apt/keyrings
```

```bash
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg && sudo chmod a+r /etc/apt/keyrings/docker.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

```bash
sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Start Docker, and let your user run it without typing `sudo` every time:

```bash
sudo systemctl enable --now docker && sudo usermod -aG docker $USER
```

That group change only applies after a full restart. **In PowerShell:**

```powershell
wsl --shutdown
```

Reopen Ubuntu and verify:

```bash
docker run --rm hello-world
```

Success looks like a paragraph beginning "Hello from Docker!". If you instead get
`permission denied`, the `wsl --shutdown` didn't fully take — close every Ubuntu window and repeat it.

---

## Step 8 — Give Docker access to the GPU ⭐

**What this does:** Installs the NVIDIA Container Toolkit, the bridge that lets a container use
your graphics card. **This is the most important checkpoint in the guide.** If it fails, Gazebo
falls back to software rendering and the simulation will be too slow to develop against.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
```

```bash
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
```

```bash
sudo apt update && sudo apt install -y nvidia-container-toolkit
```

Wire it into Docker and restart Docker:

```bash
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

**The checkpoint** — this starts a throwaway container and asks it what GPU it can see:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

You should get the familiar NVIDIA table listing **RTX A4500 Laptop GPU**. If you do, the hardest
part of this setup is behind you.

---

# PHASE D — The competition simulation

## Step 9 — Download the competition code

**What this does:** Clones (downloads) the official repository and switches to release v1.0.3.

```bash
mkdir -p ~/erc && cd ~/erc && git clone https://github.com/dfl-rlab/erc_sim_2026.git
```

```bash
cd ~/erc/erc_sim_2026 && git checkout v1.0.3
```

> **Use v1.0.3.** Released 24 Aug 2026, it fixes a bug where the gripper could not grasp the books
> at all. On an older version you would be debugging a problem that isn't yours.

Check periodically for newer releases:

```bash
git fetch --tags && git tag -l
```

Make a shortcut from Linux to your Windows-side documents folder, so you can reach the PDFs and
notes from inside Ubuntu:

```bash
ln -s "/mnt/d/Projects/Emirates Robotics Competition" ~/erc-docs
```

Now `ls ~/erc-docs` shows your PDFs, `NOTES.md`, and this guide.

---

## Step 10 — Build and launch the simulation

**What this does:** `up.sh --build` assembles the competition Docker image. This is a long,
one-time download — it took roughly 25 minutes here. Leave it running.

```bash
cd ~/erc/erc_sim_2026 && ./docker/up.sh --build
```

Final image: `erc-2026:humble-harmonic`, about 5.9 GB on disk (~6.3 GB total with the
`osrf/ros:humble-desktop` base layer and build cache).

`up.sh` auto-detects the GPU and layers in `docker-compose.gpu.yml`. You want to see:

```
[up] NVIDIA GPU + container toolkit detected — enabling GPU passthrough
```

If it says *"No usable NVIDIA runtime — falling back to software/integrated rendering"*,
stop and revisit Step 8.

### ⚠️ On WSL, GPU passthrough alone is not enough

Even with `docker-compose.gpu.yml` applied, the container rendered with **llvmpipe**:

```
Device: llvmpipe (LLVM 15.0.7)     Accelerated: no
```

The container has `d3d12_dri.so` and the CUDA libraries, but `/usr/lib/wsl/lib` inside it
holds **only `libdxcore.so`**. The NVIDIA Container Toolkit injects compute libraries, not the
D3D12 *rendering* libraries — `libd3d12.so` and `libd3d12core.so` are missing, so Mesa's d3d12
driver can't reach the GPU and silently falls back to software.

Two files were added to fix this:

| File | Purpose |
|---|---|
| `docker/docker-compose.wsl.yml` | Mounts `/usr/lib/wsl` in, sets `GALLIUM_DRIVER`, `MESA_D3D12_DEFAULT_ADAPTER_NAME`, `LD_LIBRARY_PATH` |
| `docker/up-wsl.sh` | Same as `up.sh` but layers that override on top |

**Use `up-wsl.sh` on this machine instead of `up.sh`:**

```bash
cd ~/erc/erc_sim_2026 && ./docker/up-wsl.sh
```

Result inside the container:

```
Device: D3D12 (NVIDIA RTX A4500 Laptop GPU)     Accelerated: yes    (~100 FPS)
```

> **Why a separate file rather than editing `docker-compose.yml`:** forcing
> `GALLIUM_DRIVER=d3d12` on a native Linux host *breaks* rendering, because no D3D12 layer
> exists there. Judges may well run native Linux. Keeping it opt-in means our repo stays
> correct for them while working here. **Do not merge it into `docker-compose.yml`.**

To verify rendering inside the container at any time:

```bash
docker exec erc_sim bash -c "apt-get update -qq && apt-get install -y -qq mesa-utils && glxinfo -B | grep -i 'device:\|accelerated'"
```

When the container is up, open a shell *inside* it:

```bash
./docker/attach.sh
```

Your prompt changes to show you're now inside the container. Everything from here until you type
`exit` runs in the ERC environment, not in Ubuntu.

**Inside the container**, compile the packages:

```bash
colcon build --symlink-install && source install/setup.bash
```

- `colcon build` compiles the ROS 2 packages.
- `--symlink-install` links files instead of copying them, so edits to Python files take effect
  without rebuilding. The repo's README says to always use it.
- `source install/setup.bash` tells this terminal where the freshly built programs are.
  **You must run `source install/setup.bash` in every new container shell.** Forgetting it is
  the #1 cause of "package not found" errors.

> ### ⚠️ Recreating the container deletes your build
>
> `/opt/erc_ws/src` is a bind mount from the host repo, so **source code always survives**.
> But `install/`, `build/`, and `log/` live in the container's writable layer, and
> `up.sh` / `up-wsl.sh` run `docker compose down` first — which destroys that layer.
>
> After any `up-wsl.sh`, `/opt/erc_ws/` contains only `src` and every ROS package vanishes
> from `ros2 pkg list`. **This is expected, not a broken install.** Just rebuild:
>
> ```bash
> docker exec erc_sim /entrypoint.sh bash -c "cd /opt/erc_ws && colcon build --symlink-install"
> ```
>
> It takes about **15 seconds** for all 32 packages, so it isn't worth adding a volume to
> persist it. Anything else you install inside the container (`mesa-utils`, an IDE, extra
> `pip` packages) is also lost on recreation — reinstall or add it to the Dockerfile.
>
> To restart the container *without* losing the build, use `docker start erc_sim`
> rather than `up-wsl.sh`. Verified: `install/`, `build/` and `log/` all survive a restart;
> only recreation destroys them.
>
> **But the running simulation never survives.** `restart: unless-stopped` restores the
> container, whose main process is just `bash` — anything started with `docker exec`
> (including `ros2 launch`) is gone. After any restart you get an *empty* container and
> must relaunch:
>
> ```bash
> docker exec -d erc_sim /entrypoint.sh bash -c "source /opt/erc_ws/install/setup.bash && exec ros2 launch erc_bringup simulation.launch.py > /tmp/sim.log 2>&1"
> ```
>
> This is why the Gazebo window can vanish on its own: WSL idles out → distro stops →
> container is SIGKILLed → restart policy brings back a bare container with no simulation.
> **Keeping an Ubuntu terminal open prevents the whole chain.**
>
> When checking whether Gazebo is alive, use a bracketed pattern —
> `pgrep -f "[g]z sim"`. Plain `pgrep -f "gz sim"` matches its *own* command line and
> reports success even when nothing is running.

Now launch the simulation:

```bash
ros2 launch erc_bringup simulation.launch.py
```

Gazebo Harmonic opens showing the library: a 5-column shelf unit with 20 coloured books, a red
collection bin on a table, and TIAGo Pro standing in the green start zone.

> **Be patient on first launch.** The robot spawns after roughly 5 seconds and its controllers
> come online at about 8 seconds. A brief empty or frozen window at the start is normal.

Leave this terminal running — it *is* the simulation. Closing it stops everything.

---

## Step 11 — Confirm the simulation is healthy

Open a **second** Ubuntu window and attach to the same container:

```bash
cd ~/erc/erc_sim_2026 && ./docker/attach.sh
```

> ⚠️ Run `attach.sh` for extra terminals — **never** `up.sh` a second time.

Inside it:

```bash
source install/setup.bash
```

List the robot's channels and check the ones we care about exist:

```bash
ros2 topic list | grep -E "head_front_camera|scan_|cmd_vel|bin_contacts"
```

Confirm the camera is genuinely streaming (not just listed):

```bash
ros2 topic hz /head_front_camera/head_front_camera/color/image_raw
```

This prints a running average rate. Stop it with `Ctrl+C`.

### Measured on this machine

All interfaces came up and **every topic name in `NOTES.md` was confirmed exactly against the
live system** — including the strange doubled camera namespace, which is real:

| Topic | Rate | Notes |
|---|---|---|
| `/head_front_camera/head_front_camera/color/image_raw` | 12.5 Hz | RGB |
| `/head_front_camera/head_front_camera/depth/image_rect_raw` | — | depth |
| `/head_front_camera/head_front_camera/depth/color/points` | — | generated by the `depth_to_cloud` node |
| `/scan_front_raw`, `/scan_rear_raw` | ~2–3 Hz | LiDAR |
| `/cmd_vel`, `/odom`, `/base_imu`, `/bin_contacts` | — | present |

48 topics total. All seven controllers configured and activated at startup: `head_controller`,
`torso_controller`, `arm_left_controller`, `arm_right_controller`,
`gripper_left_controller_raw`, `gripper_right_controller_raw`, `joint_state_broadcaster`.

> **`/erc/shelf_column_identification` and `/erc/shelf_row_identification` do not exist yet.**
> That is correct — they are *ours* to create. Our solution node publishes them; nothing in the
> stock simulation provides them. (Resolves the open question in `NOTES.md` §12.4.)

### ⚠️ Real-time factor is ~0.45

The simulation runs at roughly **half wall-clock speed**. Sampled over 20 s it fluctuates
between **0.35 and 0.53**:

```
real_time_factor: 0.35 … 0.53    step_size: 2 ms (500 Hz physics)
```

> A single instantaneous sample right after launch once read `0.999`. That is a warm-up
> artifact before physics load ramps up — don't trust one sample, take several.

Those sensor rates are therefore *correct in simulation time* — the camera is a 25 Hz sensor
delivering 12.5 frames per wall-clock second, and the LiDAR likewise. Nothing is misconfigured.

What's consuming the time:

| Process | CPU |
|---|---|
| `gz sim server` | ~180% |
| `gz sim gui` | ~130% |
| `depth_to_cloud` | ~97% |

Host load was 4.8 across 24 logical cores, so this is **single-thread bound**, not core-starved
— Gazebo's DART physics is largely serial. Levers worth trying, cheapest first:

1. **Windows power plan is "Balanced"** on a 2.1 GHz base clock. Switching to *High performance*
   (Settings → System → Power & battery → Power mode) is free and usually the biggest win.
2. **`depth_to_cloud` costs ~1 core** to build the point cloud. If perception ends up using only
   RGB + raw depth, disabling that node reclaims it.
3. **Run headless** (no `gz sim gui`) for automated test runs; keep the GUI only for recording
   the deliverable video.

RTF 0.5 is workable — it mainly makes iteration slower. Worth remembering the tie-breaker is
completion time, so measure timings in **simulation time**, not wall clock, when comparing runs.

Finally, open the robot's-eye view:

```bash
rviz2
```

RViz shows what the robot perceives — camera image, laser scans, its model. This will be your
main debugging window for the next three weeks.

---

# PHASE E — Team AVAA's workspace

## Step 12 — Create the AVAA solution package

**What this does:** Creates the ROS 2 package that will hold all of your team's code. Everything
you write for the competition goes here.

The judges will clone one repository and run one command:

```bash
ros2 launch avaa_solution solution.launch.py shelf_column_number:=2 book_colour:=red
```

So your package must be named consistently and must accept those two arguments exactly.

**Recommended structure:** put `avaa_solution` inside the competition repo's `src/` folder, then
push that whole repo as Team AVAA's GitHub repository. That way judges clone one thing and get
both the simulation and your solution. (We can revisit this when we set up GitHub.)

**Inside the container**, first confirm where the workspace actually is:

```bash
pwd && ls
```

You should see a `src` folder. Then:

```bash
cd src && ros2 pkg create --build-type ament_python avaa_solution
```

Then rebuild so ROS 2 knows about it:

```bash
cd .. && colcon build --symlink-install && source install/setup.bash
```

### What exists now

```
src/avaa_solution/
├── package.xml                     deps declared (rclpy, std_msgs, sensor_msgs, geometry_msgs)
├── setup.py                        installs launch/ and registers the `mission` executable
├── launch/solution.launch.py       THE entry point the judges run
└── avaa_solution/mission_node.py   state machine + the two scoring publishers (stub)
```

**Verified working — this is the command the committee will run:**

```bash
ros2 launch avaa_solution solution.launch.py shelf_column_number:=2 book_colour:=red
```

```
[INFO] [launch.user]: [AVAA] target column marker=2  book colour=red
[INFO] [avaa_mission]: AVAA mission ready — target column marker 2, book colour 'red'
[WARN] [avaa_mission]: Mission logic not implemented yet.
```

Both argument names are declared with `choices=`, so invalid input fails immediately and
visibly rather than starting a run that cannot score:

- `book_colour:=purple` → `not valid. Valid options are: ['red', 'blue', 'green', 'yellow']`
- omitting either argument → `missing required argument`

> **Why this mattered enough to do first.** `NOTES.md` §12.6 flags it: the two argument names
> are mandatory, and getting them wrong means the submission is *not evaluated at all*.
> Cheap to get right, catastrophic to get wrong — so it is nailed down and tested before any
> real logic exists.

One deliberate omission: `mission_node.py` advertises `/erc/shelf_column_identification` and
`/erc/shelf_row_identification` but **does not publish to them yet**. Publishing a wrong
column or row is worse than publishing nothing, so the `report_column()` / `report_row()`
helpers are there and will be called once perception can actually justify a value.

Perception, navigation, and manipulation come next as separate nodes — the rubric explicitly
rewards modularity over one large file.

---

## Checkpoint summary

Tick these off as you go. If something fails, note which step and the exact error text.

Verified on this machine 2026-08-25.

| # | Checkpoint | Status | Evidence |
|---|---|---|---|
| 1 | WSL2 + Ubuntu installed | ✅ | WSL 2.7.12.0, Ubuntu 24.04.4 LTS, VERSION 2 |
| 1b | Linux user account created | ✅ | user `aabde`, uid 1000, in `sudo` group |
| 3 | Linux lives on D: | ✅ | `wsl\Ubuntu-24.04\ext4.vhdx`, nothing on C: |
| 4 | systemd running | ✅ | `is-system-running` → `running`, PID 1 = systemd |
| 5 | Base tools installed | ✅ | git 2.43.0, gcc 13.3.0, curl 8.5.0 |
| 6 | GPU accelerated in Linux | ✅ | `D3D12 (NVIDIA RTX A4500)`, `Accelerated: yes`, ~110 FPS |
| 6 | Windows can display Linux apps | ✅ | WSLg, `DISPLAY=:0`, X0 socket present |
| 7 | Docker works without sudo | ✅ | Docker 29.7.2, Compose v5.5.0, `hello-world` ran |
| 8 | **GPU works in Docker** ⭐ | ✅ | A4500, 16 GB VRAM visible via `--gpus all` |
| 9 | Competition repo cloned | ✅ | `~/erc/erc_sim_2026` at tag `v1.0.3` |
| 10 | Simulation image built | ✅ | `erc-2026:humble-harmonic`, 5.9 GB |
| 10 | **GPU rendering in container** ⭐ | ✅ | `D3D12 (RTX A4500)`, `Accelerated: yes`, ~100 FPS — needs `up-wsl.sh` |
| 10 | Workspace builds | ✅ | 32 packages in ~15 s |
| 10 | Simulation runs | ✅ | `gz sim server` + `gz sim gui`, 7 controllers active |
| 11 | Interfaces live | ✅ | 48 topics; all `NOTES.md` names confirmed exact |
| 11 | Real-time factor | ⚠️ | 0.50 — usable, see notes above |
| 12 | AVAA package builds | ✅ | `avaa_solution` builds in 1.3 s |
| 12 | **Judges' command runs** ⭐ | ✅ | `ros2 launch avaa_solution solution.launch.py shelf_column_number:=2 book_colour:=red` |
| 12 | Bad arguments rejected | ✅ | `book_colour:=purple` and missing args both fail loudly |

### Two WSL lifecycle quirks to know

- **WSL stops the distro when no terminal is attached**, which SIGKILLs the container
  (exit 255). `vmIdleTimeout=-1` in `C:\Users\aabde\.wslconfig` does *not* prevent this —
  measured. What fixes it is `restart: unless-stopped` in `docker-compose.wsl.yml`, which
  brings the container back when WSL next starts. **Keep one Ubuntu terminal open while working.**
- **Recreating the container wipes `install/` and `build/`** — see the warning in Step 10.

### Environment-specific notes for this machine

- **Ubuntu user:** `aabde` · **Workspace:** `~/erc/erc_sim_2026` · **Docs shortcut:** `~/erc-docs`
- **Two env vars are load-bearing** for GPU rendering — see Step 6. Without them you silently
  get software rendering or the wrong GPU.
- `x11-xserver-utils` had to be installed separately: `up.sh` calls `xhost` but hides the error
  if it's missing, so the failure would have been invisible.
- `/dev/dri` does not exist in WSL (it uses `/dev/dxg`). `docker-compose.yml` mounts `/dev/dri`
  unconditionally; Docker just creates an empty directory, and `privileged: true` gives the
  container `/dev/dxg` anyway. Harmless — don't "fix" it.

---

## Daily workflow — the `sim` command

A helper is installed at `~/.local/bin/sim`. It handles the container, the build, and the
launch, so you don't have to remember which shell you're in or what state things are in.
Run it from **any Ubuntu terminal, any directory**:

| Command | What it does |
|---|---|
| `sim start` | Launch Gazebo. Starts the container and builds the workspace first if needed. |
| `sim stop` | Stop Gazebo, leave the container up. Waits for a clean shutdown. |
| `sim restart` | Stop then start. |
| `sim shell` | Shell inside the container with ROS 2 already sourced. |
| `sim status` | What's running: container, build, Gazebo, current real-time factor. |
| `sim log` | Follow the simulation log. |
| `sim build` | Rebuild the ROS workspace. |
| `sim rebuild` | Recreate the container from scratch (destroys `install/` and `build/`). |

**When something looks wrong, run `sim status` first.** It tells you which of the three
layers — container, build, simulation — is actually missing.

---

## Troubleshooting

### `[WARN:COPY MODE]` in the Gazebo title bar — harmless ✅

The window title reads:

```
[WARN:COPY MODE] Gazebo Sim (Ubuntu-24.04)
```

**This is not a Gazebo message and not an error.** It is added by **WSLg**, the Windows
component that shows Linux windows on your desktop. It means WSLg is *copying* the finished
window image across to Windows instead of using a zero-copy shared-surface path. It describes
how the picture is delivered, **not how it was drawn**.

Proof it has nothing to do with our setup, Docker, or Gazebo: running plain `glxgears`
directly in Ubuntu — no container involved — produces

```
[WARN:COPY MODE] glxgears (Ubuntu-24.04)
```

Every WSLg window on this machine gets the prefix. And that same glxgears run held **~110 FPS
on the RTX A4500**, which is only possible with hardware rendering. The 3D itself is
GPU-accelerated; we verified `GL_RENDERER = D3D12 (NVIDIA RTX A4500 Laptop GPU)` in Gazebo's
own Ogre log.

There is no fix to apply and nothing is degraded enough to matter. **Ignore it.**

### `[WARN:COPY MODE]` disappeared after a reboot

Following a Windows restart the title came back as plain `Gazebo Sim (Ubuntu-24.04)` with no
prefix, so WSLg found the accelerated presentation path on that boot. The prefix is not
sticky and may return. It was never the cause of any problem here — see the entry above.

### "I can see the simulation but can't interact with it"

Different cause from the window not opening. Check whether Gazebo actually has focus:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "D:\Projects\Emirates Robotics Competition\tools\restore-gazebo.ps1"
```

Observed once: every Gazebo health check passed — `responding=True`, `IsHungAppWindow=False`,
`IsWindowEnabled=True`, message pump answering in 14 ms — but `frontmost=False`, because a
**Windows Security** window (1298×1010 at 100,22) was covering it and holding focus. Clicks
and keystrokes went there instead.

A reboot starts Docker and WSL networking, which can raise a firewall prompt. **Read that
window before dismissing it** rather than burying it under Gazebo.

Diagnostic order for "can't interact":

1. `sim status` — is the simulation actually running and is the RTF advancing?
2. `restore-gazebo.ps1` — is `frontmost` false? Something else owns the focus.
3. `IsHungAppWindow` — if true, the GUI is genuinely wedged; `sim restart`.

### Gazebo appears in the taskbar but the window won't open — fixed ✅

**This was the real bug**, and the `[WARN:COPY MODE]` title made it look like the two were
related. They are not.

When Gazebo is launched from a **detached** process (`docker exec -d`, which is what
`sim start` does), WSLg maps the window **minimized**, and clicking its taskbar button does
not reliably restore it. Measured state:

```
visible   : True
minimized : True
rect      : L=-32000 T=-32000  (W=160 H=28)
```

`-32000,-32000` at 160×28 is Windows' placeholder geometry for a minimized window. The
window genuinely existed and Gazebo was running fine — it simply could not be brought
to the screen.

**Fixed automatically.** `sim start` now calls `tools/restore-gazebo.ps1`, which
un-minimizes the window and, if any part of it is off-screen, repositions it to 60,40 at
1400×900. A second failure mode it handles: Gazebo sometimes restores with `Top = -32`, so
the title bar sits above the screen edge and cannot be dragged.

To run it by hand at any time:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "D:\Projects\Emirates Robotics Competition\tools\restore-gazebo.ps1"
```

> Launching Gazebo in the **foreground** from your own terminal
> (`sim shell`, then `ros2 launch erc_bringup simulation.launch.py`) does not hit this —
> the window maps normally. It is specific to detached launches.

There were **two** stacked problems here, and fixing only the first is not enough:

1. **Minimized** at `-32000,-32000` — solved by `ShowWindow(SW_RESTORE)`.
2. **Buried behind other windows.** Once un-minimized the window was correctly placed at
   60,40 but still invisible, because Windows refuses `SetForegroundWindow` from a process
   that is not itself in the foreground. The call silently does nothing.

The second is solved by briefly setting the window topmost and then clearing that flag,
which raises it without needing foreground rights. The script reports whether it worked:

```
restore-gazebo: window at 60,40 (1400x900) minimized=False frontmost=True
```

If it ever prints `frontmost=False`, one click on the Gazebo taskbar button finishes the job —
the window is on-screen and restored by then, just not focused.

### Other

**"The Windows Subsystem for Linux is not installed"**
Step 1 wasn't run as Administrator, or you skipped the reboot.

**Password isn't working in Ubuntu**
Characters genuinely don't display while typing `sudo` passwords. Type it blind and press Enter.
If you've lost it: in PowerShell run `ubuntu2404.exe config --default-user root`, open Ubuntu,
run `passwd YOUR_USERNAME` to set a new one, then set the default user back.

**`glxinfo` shows llvmpipe (software rendering)**
The machine has Intel UHD graphics alongside the A4500 and WSL sometimes picks the wrong one.
Confirm `nvidia-smi` works in PowerShell, run `wsl --update`, then `wsl --shutdown` and reopen.

**Gazebo window never appears**
WSL provides the display through WSLg. If the container can't reach it, try `xhost +local:`
in Ubuntu (not in the container) before running `up.sh`, then relaunch. Check the terminal for
errors mentioning `DISPLAY` or `X11`.

**Docker says "permission denied" on the socket**
The `docker` group change from Step 7 needs a complete restart. Close all Ubuntu windows, run
`wsl --shutdown` in PowerShell, and reopen.

**Docker isn't running after a reboot**
Check systemd took (Step 4). As a stopgap: `sudo service docker start`.

**"Package 'avaa_solution' not found"**
You forgot `source install/setup.bash` in that terminal. Every new container shell needs it.

**Disk usage keeps climbing**
The VHDX grows but never shrinks on its own. Reclaim space with `wsl --shutdown` followed by
`Optimize-VHD` (needs Hyper-V tools) or `diskpart` compact. Inside Ubuntu,
`docker system prune -a` clears unused images and build cache — but note this will force a
re-download if you remove the ERC image.

**Everything is mysteriously slow**
Confirm your code is under `~/erc/` and *not* under `/mnt/d/`. Working directly across the
Windows/Linux boundary is drastically slower.

---

## What comes next

Once every checkpoint passes, the build order is:

1. **GitHub** — create the Team AVAA repository and push (deliverable D1 needs it)
2. **Perception** — read the camera, detect the overhead column markers 1–5 and book colours
3. **Navigation** — Nav2 to drive from start zone to the correct column and back
4. **Manipulation** — MoveIt 2 to grasp a book and place it gently in the bin (+4 vs +2 for a drop)
5. **Scoring plumbing** — publish to `/erc/shelf_column_identification` and
   `/erc/shelf_row_identification`, and save timestamped annotated images to `erc_images/`
6. **Deliverables** — the 5-minute unedited video and the 5-page report

See `NOTES.md` for the full rules, scoring table, and deliverable requirements.
