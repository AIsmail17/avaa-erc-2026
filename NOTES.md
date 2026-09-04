# Emirates Robotics Competition 2026 (4th Edition) — Project Notes

**Source docs:** `Emirates_Robotics_Competition___Edition_4__Overview_.pdf`, `Emirates_Robotics_Competition___Edition_4__Phase_1_.pdf` (both v1.0, dated 05/08/2025)
**Notes compiled:** 2026-08-25

---

## 1. What the project is

Build the **software stack for a "Library Assistant Robot"** — a mobile manipulator that autonomously
locates, picks, and delivers a requested book in a library setting.

- **Venue:** Mohammed Bin Rashid Library (MBRL), Dubai
- **Platform:** TIAGo Pro mobile manipulator (PAL Robotics) — hardware is *given*, teams write software only
- **Audience:** undergraduate teams from local universities
- **Framework:** ROS 2 Humble
- **Organizers/sponsors:** Dubai Future Labs (DFL), Rochester Institute of Technology (RIT), MBRL, Khalifa University (KU), American University of Sharjah (AUS)

### Two phases

| Phase | What | Where |
|---|---|---|
| **Phase 1 — Simulation** | Develop + test in Gazebo, submit code/video/report | Provided Docker image |
| **Phase 2 — Physical Deployment** | Run the *same* code on the real TIAGo Pro | MBRL, in person |

Top teams from Phase 1 qualify for Phase 2. Phase 2 technical details are released only after the Phase 1 deadline.

### Prizes

- 1st: **AED 15K** · 2nd: **AED 10K** · 3rd: **AED 5K** (AED 30K total)
- Plus partial RIT master's scholarships (GPA thresholds and eligibility TBA)
- **Condition of accepting prize money:** winners must open-source and document their code (GitHub/GitLab etc.) so anyone can replicate and build on it.

---

## 2. The problem → the solution pipeline

A classic **perception → navigation → manipulation → navigation → placement** loop.
The solution is invoked with two runtime arguments and must run fully autonomously, no manual intervention.

```bash
ros2 launch YOUR_TEAM_PACKAGE_NAME solution.launch.py shelf_column_number:=2 book_colour:=red
```

**Required sequence (a single challenge trial):**

1. **Start** in the Start/End Zone — the starting pose is *not* guaranteed to face the shelves, so the robot must self-orient.
2. **Vision: identify the target shelf column.** A numeric marker (1–5) is placed **overhead** on each column, and marker placement is **randomized on every simulation load** — so `shelf_column_number:=2` maps to a different physical location each run.
3. **Navigate** to that column and position within arm's reach of the books.
4. **Vision: identify the target book by colour** (red / blue / green / yellow) and determine its **row (1–4)**.
5. **Grasp** the book with the PAL 2-finger parallel gripper — **one arm only**.
6. **Navigate back** to the Start/End Zone with the book in hand.
7. **Vision: identify the red collection bin** and **place** the book inside it.

### Key design implication

Both the column marker and the colour→row assignment are randomized per run. The two launch arguments are
**semantic** (which column *label*, which *colour*), not spatial. The entire spatial solution must be derived
online from the RGB-D camera and LiDAR — **hardcoded coordinates cannot work**, which is exactly what the
randomization is designed to enforce.

---

## 3. Simulation environment spec

| Element | Details |
|---|---|
| **Shelves unit** | 5 columns × 6 rows, ~3 m from start. **210 (h) × 525 (l) × 30 (d) cm**. All columns/rows fixed and reachable. |
| **Column markers** | Numeric marker **1–5**, placed **overhead**, randomized each load |
| **Books** | Only the **middle 4 rows** are stocked → **20 books** (5 cols × 4 rows). Four colours, **exactly one of each colour per column**. |
| **Book randomization** | Horizontal position randomized to **25–75% of row width**; colour→row assignment randomized **vertically** (same colour is not always on the same row between runs) |
| **Book physical** | **25 × 16 × 3 cm**, **300 g**, fits the gripper |
| **Collection bin** | Single **red** bin on a table at fixed reachable height. Bin **50 × 31 × 21 cm**; table **140 × 80 × 73 cm** |
| **Start/End Zone** | Green area **0.8 × 0.8 m**. The bin is reachable from here *if* the robot is rotated to the correct orientation. |

---

## 4. Hardware — TIAGo Pro (fixed, no modifications allowed)

- **Mobile base:** 4 mecanum wheels, full omnidirectional motion. Max **1 m/s** fwd/back, **0.7 m/s** diagonal
- **LiDAR:** 2 sensors (front + rear), combined **360° FoV**, **10 m** range
- **Lifting torso:** 1 DoF, **35 cm** vertical travel
- **Head:** 2 DoF (pan/tilt), **RGB-D camera** (RealSense D435i per repo README) + 10" touchscreen
- **Arms:** 2 × **7 DoF**, 92 cm vertical reach, 236 cm combined horizontal reach, **3 kg** payload, 2-finger PAL parallel gripper. **Only one arm may be used for this challenge.**
- **Robot PC:** Intel i5, **16 GB RAM**, 512 GB SSD, Ubuntu LTS + ROS 2 + PAL SDK

> **No hardware modifications, no additional sensors.** All work is software-level in ROS 2.
> Compute budget on the real robot is modest (i5 / 16 GB, no dedicated GPU mentioned) — worth weighing
> before committing to heavy deep-learning perception that must also run in Phase 2.

---

## 5. Software — the provided ERC 2026 Docker image

**Contents (must not be modified):**

- Ubuntu **22.04** + ROS 2 **Humble**
- Gazebo **Harmonic**, competition world pre-loaded
- Simulated TIAGo Pro with **identical ROS 2 interfaces to the physical robot** (same topics, controllers, sensor outputs)
- **RViz 2**
- **Nav2** and **MoveIt 2** pre-installed and configured for TIAGo Pro
- Base bringup launch file:

```bash
ros2 launch erc_bringup simulation.launch.py
```

> You may **not** change the OS, ROS 2 distro, Gazebo version, or robot model. You *may* install extra tools
> (IDEs, etc.) inside the container. **Any submission that doesn't run correctly inside the stock Docker image
> will not be evaluated** — so test on multiple machines before submitting.

### Host requirements (from repo README)

- **x86_64 Linux host** (Ubuntu 22.04/24.04 recommended) with **X11** — ARM is not supported
- Docker Engine + Docker Compose v2, Git
- ~**15 GB** free disk
- NVIDIA GPU + `nvidia-container-toolkit` optional but recommended for rendering performance
- Default `ROS_DOMAIN_ID` = **23**; dependencies are vendored (no network access needed after clone)

### Setup sequence

```bash
./docker/up.sh --build
```

```bash
./docker/attach.sh
```

```bash
colcon build --symlink-install && source install/setup.bash && ros2 launch erc_bringup simulation.launch.py
```

---

## 6. ROS 2 interfaces

### Camera topics (from the Phase 1 doc — note the doubled namespace)

| Stream | Topic |
|---|---|
| RGB | `/head_front_camera/head_front_camera/color/image_raw` |
| Depth | `/head_front_camera/head_front_camera/depth/image_rect_raw` |
| Point cloud | `/head_front_camera/head_front_camera/depth/color/points` |

### Scoring topics you must publish to

| Purpose | Topic | Type |
|---|---|---|
| Shelf column ID | `/erc/shelf_column_identification` | `std_msgs/msg/Int32` |
| Shelf row ID | `/erc/shelf_row_identification` | `std_msgs/msg/Int32` |

These are **monitored live by the organizing committee** during evaluation.

### Other interfaces (from repo README)

| Purpose | Topic |
|---|---|
| Trial end / bin contact | `/bin_contacts` |
| Base velocity | `/cmd_vel` (`geometry_msgs/Twist`) |
| Odometry | `/odom` |
| Front / rear LiDAR | `/scan_front_raw`, `/scan_rear_raw` |
| IMU | `/base_imu` |
| Head / torso | `/head_controller/joint_trajectory`, `/torso_controller/joint_trajectory` |
| Arms | `arm_left_controller` / `arm_right_controller` (JointTrajectory) |
| Grippers | `gripper_left_controller_raw` / `gripper_right_controller_raw` |

### Annotated image output

- Saved by **your ROS 2 node during the trial**, from the **live camera feed**, into a folder named **`/erc_images/`** in your repository
- Each image **must carry a timestamp** so judges can verify it was captured during the run
- Manually edited images, or images produced outside the trial run, **will not be accepted**

---

## 7. Scoring criteria (same for both phases)

| Behaviour | Points |
|---|---|
| Identify correct shelf column — by publishing to ROS 2 topic | **+1** |
| Identify correct shelf column — by saving image with bounding box around target column | **+2** |
| Navigate to correct shelf column *(awarded on successful grasp)* | **+3** |
| Identify shelf row (1–4) — by publishing to ROS 2 topic | **+1** |
| Identify shelf row (1–4) — by saving image with bounding box around target book | **+2** |
| Grasp and pick up the target book | **+3** |
| Navigate to collection bin with book in hand *(awarded on successful delivery)* | **+3** |
| Place book in bin — **dropped** in | **+2** |
| Place book in bin — **gently placed** in | **+4** |
| Collision with shelf or other objects (**per collision**) | **−0.5** |

- Identification points are **cumulative**: topic (+1) *and* image (+2) = **+3** max per identification task
- Placement points are **mutually exclusive**: dropped **or** gently placed, not both
- **Maximum achievable: 19 points** (3 + 3 + 3 + 3 + 3 + 4)
- Total score has a **floor of 0** — you cannot finish negative
- **Tie-breaker: fastest completion time.** Timer starts when `solution.launch.py` launches and ends when the book contacts the bin (detectable via `/bin_contacts`)

### Collision rules

- A collision is any *unintended* contact between TIAGo Pro (base, torso, arm, or gripper) and the shelves, books, table, or bin
- Gripper ↔ target-book contact is **not** penalized (intentional)
- Continuous unbroken contact with the same object counts as **one** event (−0.5)
- A new penalty only triggers after contact fully stops and a separate contact occurs, with a **1 s cooldown** on the same object
- Detected by Gazebo contact sensors in Phase 1; by judges' direct observation in Phase 2

---

## 8. Phase 1 deliverables

Submitted via a **Phase 1 Deliverables Submission Form**, released closer to the deadline.

### D1 — Code

- GitHub repository link
- Must contain all ROS 2 packages, including the solution package holding `solution.launch.py`
- `README.md` describing the packages the team created
- All dependencies declared in `package.xml`
- Judged on **organization** (comments explaining functions/sections), **modularity** (perception / planning / execution as distinct nodes vs. one big file), and **error handling** (detecting and recovering from missed grasps, lost topics, unexpected sensor input)

### D2 — Video

- YouTube link, **max 5 minutes**
- **Unedited**, no cuts, **not sped up**
- **Team name visible before the trial begins**
- Must **start by launching `simulation.launch.py`, then `solution.launch.py`**
- On-screen **timer** from trial start to finish
- Must show the solution successfully completing all tasks

### D3 — Report

PDF, **max 5 pages** excluding title page. Must follow this exact structure:

1. **System architecture** — diagram of ROS 2 nodes and their roles; how perception, navigation, and manipulation connect
2. **Perception** — how you detect the shelf column number and the book colour, and *why* this approach; **quantitative** accuracy across a number of trials
3. **Navigation** — how the robot reaches the column and the bin; **quantitative** data on minimizing travel time; **RViz screenshots of planned vs. executed path**
4. **Manipulation** — grasp and place approach, arm/gripper positioning relative to the book, and how bin placement is executed
5. **Results** — scores across **5 simulation trials**, detection and grasping success rates, average trial time
6. **Limitations and Future Work** — challenges faced and how they were addressed, what you'd improve with more time

---

## 9. Qualification rubric (Phase 1 → Phase 2)

| Component | Weight |
|---|---|
| Evaluation Challenge Trial | **50%** |
| D1: Code | **20%** |
| D2: Video | **10%** |
| D3: Report | **20%** |

**Evaluation trial:** the committee clones your repo and runs your `solution.launch.py`. **Best of three
attempts** counts. It must perform consistently with **no manual intervention**, under the same conditions
and in the same environment as your submitted video.

Note that the trial is 50% but the written deliverables are the other 50% — a mediocre run with an
excellent report and clean modular code can still out-rank a better run with weak documentation.

---

## 10. Timeline

| Milestone | Date |
|---|---|
| Registration deadline | June 30, 2026 *(passed)* |
| **Simulation Phase deadline** | **September 15, 2026** |
| TIAGo Pro trial sessions | October 2026 (TBA) |
| Competition Day | November 2026 (TBA) |

**Competition Day format:** a 10-minute slot per team, **two trials**, highest score determines final ranking.
Trial-session schedule and duration will be communicated after Phase 1 results are announced.

---

## 11. Data, resources and links

| Resource | Link / location |
|---|---|
| **Competition repo (Docker image + packages)** | https://github.com/dfl-rlab/erc_sim_2026 |
| Releases — download the `.zip` of the **latest** | https://github.com/dfl-rlab/erc_sim_2026/releases |
| Overview PDF | `Emirates_Robotics_Competition___Edition_4__Overview_.pdf` |
| Phase 1 PDF | `Emirates_Robotics_Competition___Edition_4__Phase_1_.pdf` |
| TIAGo Pro datasheet | Referenced as "the official TIAGo Pro datasheet" — on the PAL Robotics site |

### Repo structure

- `docker/` — `up.sh`, `attach.sh`, Docker config
- `src/erc_description` — robot URDF
- `src/erc_bringup` — launch files (`simulation.launch.py`)
- `docs/` — documentation and assets

### Releases published so far

| Tag | Date | Notes |
|---|---|---|
| **v1.0.3** | **Aug 24, 2026** | Fixes gripper not grasping books — reduced book spine width, set gripper range limits |
| v1.0.1 | Aug 17, 2026 | README updated with guidance on creating and submitting issues |
| v1.0.0 | Aug 12, 2026 | Initial release of competition packages |

> **Data note:** there is **no provided dataset**. All perception input is generated live from the simulated
> RGB-D camera and LiDAR. Any training data for digit or colour detectors has to be self-collected from the
> Gazebo environment.

---

## 12. Flags and open questions

1. **Timeline is tight.** The Simulation Phase deadline is **September 15, 2026** — about **3 weeks** from today (Aug 25).
2. **Host OS.** The repo requires an **x86_64 Linux host with X11**. On Windows that means WSL2 (with WSLg or an X server) or a separate Linux machine / dual boot. Worth sorting before anything else.
3. **Use release v1.0.3.** Published Aug 24, 2026, and it specifically fixes the gripper failing to grasp books. An older clone will fight a known-broken grasp.
4. **Underscores in the PDF.** The PDF text layer drops underscores in some fonts. The topic names above (`/erc/shelf_column_identification`, `/erc/shelf_row_identification`, `/bin_contacts`, `/erc_images/`, and the camera topics) are reconstructed to standard ROS naming — **verify each against the repo before relying on them.**
5. **`/erc_images/`** is written with a leading slash in the doc but described as a folder "in the team's repository" — worth clarifying whether it's an absolute container path or repo-relative.
6. **The two launch args are mandatory.** `shelf_column_number` and `book_colour` must be accepted exactly, or the submission is not evaluated at all. Cheap to get right, catastrophic to get wrong.
7. **Gentle placement is worth double a drop** (+4 vs +2), and a collision is only −0.5. Being careful pays more than being fast, except as a tie-break.
