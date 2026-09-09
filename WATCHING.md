# Watching a run in a Gazebo window

Two machines, and they are not equivalent any more.

**This laptop** has 24 cores, 31 GB and an RTX A4500, against the NUC's 4 threads. The
simulation runs three to four times faster here and the whole grasp can be watched on a
bench that starts in under a minute. Start here.

**The NUC** is still the machine the arena runs on when you want the competition world
end to end, and part two below is unchanged for it.

Everything below was run and checked; where a number appears it was measured, not
guessed.

---

# Part one — on this laptop

The container already has what it needs: WSLg puts a display at `:0`, the X socket is
mounted into the container, and `/dev/dxg` is passed through for the GPU. Nothing to
install and nothing to configure.

## The short version

Open **Windows Terminal** and pick the **Ubuntu-24.04** tab (or run `wsl -d Ubuntu-24.04`
from PowerShell). Then:

```bash
cd ~/erc/erc_sim_2026
HEADLESS=false ./tools/lab up dart
```

A Gazebo window opens on your Windows desktop with the robot and one book. To watch the
whole grasp happen in it:

```bash
HEADLESS=false ERC_GRASP_FIX=1 bash tools/labgrasp.sh
```

and when you are done:

```bash
./tools/lab down
```

## What the bench is, and why it is the one to watch

It is the robot, one book, and nothing else — no shelf, no markers, no walls, no
collection bin, nineteen fewer books, and no cameras. The arena costs about thirteen
minutes to reach a grasp and spends eleven of them on an approach that is already
understood. The bench reaches the same grasp in about two.

What you will see, in order. The left column is what the clock on the wall says with a
window open; the right is simulated time, which is what the log timestamps count.

| you will see | wall | simulated |
|---|---|---|
| the window opens, robot standing, one book floating in front of it | 0:30 | |
| the arm folds to the driving posture, the base is pinned | 1:30 | |
| the right arm folds away and the shelf goes into the planner | 3:00 | 0:33 |
| the torso rises to the row height | 3:50 | 0:51 |
| the arm unfolds to the pre-grasp, square to the book | 4:50 | 1:14 |
| the jaws open | 5:10 | 1:22 |
| the arm reaches in, in two stages, pausing between them | 6:30 | 1:51 |
| the jaws close | 6:40 | 1:54 |
| the arm lifts, withdraws, and stows with the book | 9:30 | 3:06 |

Those are from one measured run: 186 simulated seconds from `idle` to `done`, at a
real-time factor of about 0.37 with the window open. The factor is what decides the wall
column, and it moves with what else the machine is doing — headless it runs 0.5 to 1.0
and the whole thing takes about half as long.

The states are printed in the terminal as it goes:

    idle -> scene -> raising -> pregrasp -> opening -> advancing -> servoing
         -> clamping -> lifting -> withdrawing -> stowing -> done

`done` is the one you are waiting for. `failed` means it stopped and said why on the line
above.

## Two things that will look wrong and are not

**The book floats.** Gravity is off for the bench book on purpose. It has to stay where
the bench puts it or no arrival number means anything, and neither alternative works:
holding it up by repeated `set_pose` runs at one or two hertz because every call is a
subprocess, and a book falls 0.78 m in the 0.4 simulated seconds between placements.
Whether a real grip holds a book against gravity is measured separately, with gravity on,
by `tools/in-sim canitgrip.py`.

**The fingers pass through the book as they close.** They genuinely do, and it is the
simulator rather than the solution. `MANIPULATION.md` section 1 has the measurements. The
book still gets picked up because `sim_grasp_fix` carries it, which is scaffolding that
Phase 2 drops by not launching it.

## Watching the arena instead, here

The full competition world, with the shelf and twenty books:

```bash
cd ~/erc/erc_sim_2026
./tools/sim restart --fast          # leave --headless off and you get a window
./tools/run-once.sh 3 yellow        # column 3, a yellow book
```

`./tools/sim status` says what is running, `./tools/sim log` follows the log, and
`./tools/sim stop` ends it.

## If the window does not appear

- `echo $DISPLAY` in the Ubuntu terminal should print `:0`. If it is empty, close the
  terminal and open a new one — WSLg sets it at login.
- `docker exec erc_sim bash -c 'ls /tmp/.X11-unix'` should list `X0`. If it does not, the
  container was started without the socket mounted; `./tools/sim rebuild` recreates it.
- The real-time factor with a window on this machine is about **0.37** on the bench
  against **0.5 to 1.0** headless. A window costs roughly half the speed. That is the
  price of watching, and it is worth paying while you are actually looking.

---

# Part two — on the NUC, through remote desktop

For the machine at **192.168.1.26** (`ahmedo@nucserver`). Everything in this part was run
and checked on 2026-09-07 on that machine.

Every command in this part goes in a terminal **inside your remote desktop session**, not
over SSH. That matters for exactly one reason, explained under [Why the remote desktop
session](#why-the-remote-desktop-session).

---

## The short version

Four commands, two terminals.

```bash
cd ~/erc/erc_sim_2026 && git pull && xhost +SI:localuser:root
```

```bash
./tools/sim restart --fast --headless
```

```bash
./tools/sim gui
```

Then, in a **second** terminal, leaving the viewer open:

```bash
cd ~/erc/erc_sim_2026 && ./tools/run-once.sh
```

The rest of this file is what each of those does, what you should see, and what to do
when one of them does not do it.

---

## 1. Get the latest code and let the container draw on your screen

```bash
cd ~/erc/erc_sim_2026 && git pull && xhost +SI:localuser:root
```

`xhost` is the part people miss. Your desktop's X server only accepts connections from
programs running as **you**, and the simulator runs as **root inside Docker**, so without
this the Gazebo window simply never appears and nothing says why. Checked on the NUC:

```
$ xhost
access control enabled, only authorized clients can connect
SI:localuser:ahmedo
```

`+SI:localuser:root` adds root on this machine and nothing else. (You will also see
`xhost +local:docker` in `docker/up-wsl.sh`; that is the broader version of the same
thing. Either works.)

**It does not survive a logout.** Run it once per desktop session. To undo it:

```bash
xhost -SI:localuser:root
```

## 2. Start the simulator with no window

```bash
./tools/sim restart --fast --headless
```

Takes about a minute. It should end with:

```
Gazebo is running.
  colour camera : streaming
  joint states  : streaming
  udp frames    : none dropped (rmem_max 33554432)
```

**Starting headless and attaching a window afterwards is deliberate, not a workaround.**
In Gazebo the GUI and the physics server are one process, and the GUI is the fragile
half: when it fails to get an OpenGL context it aborts, and the abort takes the server
down with it — every controller spawner dies on "Could not successfully call service
/controller_manager/load_controller" and the run is gone before the robot moves. Counted
over one evening, that happened on roughly **a third** of GUI starts. Started separately,
the viewer is disposable: if it dies the simulation carries on at full speed and you just
run `sim gui` again.

The three lines about sensors are not decoration. A simulator that comes up "running" but
blind looks exactly like broken perception and has cost days. `sim restart` now checks
this itself and restarts if the camera never streams.

`--fast` turns off the depth point cloud, which costs about a full CPU core and which
nothing currently uses. Leave it off for watching; turn it back on for a scored run only
if manipulation ever needs the cloud.

## 3. Open the window

```bash
./tools/sim gui
```

You should see:

```
Attaching a viewer (attempt 1 of 3)...
Drawing on display :10.
Viewer running (the simulation is untouched if it crashes).
```

**Check that it says `:10`.** There is no display `:0` on this machine — xrdp runs its own
X server on `:10`, and `:1024` and `:1025` are xrdp's internals. If it says `:0` you are
running over SSH rather than in the desktop session, and the window will go nowhere.

### What it costs

Measured on the NUC with the same simulation running before and after attaching:

| | real-time factor |
|---|---|
| headless | 0.248 |
| with the viewer attached | 0.122 |

**The window roughly halves the simulation speed**, because the NUC renders it on
integrated graphics. A run that takes ten minutes headless takes twenty with the window
open. That is fine for watching and wrong for collecting results — close the viewer when
you are gathering numbers.

To close it without touching the simulation:

```bash
docker exec erc_sim pkill -f 'gz sim -g'
```

## 4. Run the robot

In a **second terminal**, leaving the viewer open:

```bash
cd ~/erc/erc_sim_2026 && ./tools/run-once.sh
```

It restarts the simulator first — so if you want to keep the viewer you already have,
skip `run-once.sh` and launch the mission directly:

```bash
docker exec -it erc_sim /entrypoint.sh bash -c \
  'source /opt/erc_ws/install/setup.bash && \
   ros2 launch avaa_solution solution.launch.py shelf_column_number:=3 book_colour:=red'
```

`shelf_column_number` and `book_colour` are the task: which marker digit to look for and
which colour book to fetch.

---

## What you should see, and roughly when

Times are **simulated** seconds, from the mission clock. Multiply by 1/RTF for wall clock
— at 0.12 with the window open, 157 simulated seconds is about twenty minutes.

| Sim time | What happens |
|---|---|
| 0–30 s | Both arms fold to the driving posture. Nothing else moves. |
| 30–50 s | The base turns on the spot, head tilted up, hunting for its column marker. It may reverse a metre or so first — the markers are at 2.26 m and cannot be read from close in. |
| ~50 s | Turns to face its column, then drives in, slowing and sliding sideways to line up. |
| ~50 s | `phase approach -> grasp`. The robot is squared to the shelf about 0.65 m out. |
| 50–160 s | The torso rises, the left arm unfolds, reaches into the shelf, closes, withdraws. |
| ~157 s | `phase grasp -> deliver`. The arm folds in with the book and the base goes looking for the red bin. |

**As of 2026-09-07 the run does not finish.** The approach and the reach both work — the
gripper arrives within a millimetre of the book — but the jaws close on air and the book
stays on the shelf, so the delivery drives off empty and fails. `STATE.md` has the
current detail. Everything up to and including the reach is worth watching; the delivery
is not, yet.

---

## When something does not work

**"Container not running" — but it is.** The first thing to suspect is this terminal
rather than the container:

```bash
docker ps
```

If that says *permission denied*, the shell is not in the `docker` group even though the
account is. Adding a user to a group changes `/etc/group` at once and changes nothing
about sessions already logged in, so an ssh login made afterwards works while the desktop
session started that morning does not — same machine, same user, different answer.

```bash
newgrp docker
```

fixes the terminal you type it in. Logging out of the desktop session and back in fixes
all of them. `sim` now says which of the two it is instead of reporting the container
missing and offering to rebuild it, which is what it used to do — and rebuilding destroys
the workspace build for a problem that was never the container.

**The window never appears.** Almost always `xhost` (step 1) or the wrong display. Check
which display it chose, and look at what the viewer said:

```bash
docker exec erc_sim tail -30 /tmp/gui.log
```

`drisw`, `swrast` or "Failed to create OpenGL context" mean the GPU path failed. The
simulation is untouched; use the browser view below.

**The window appears and then dies.** `sim gui` retries three times by itself. If all
three fail, the simulation is still running — nothing is lost.

**Nothing happens after the arms fold.** Check the simulator is not blind:

```bash
./tools/sim status
```

If `colour camera` says SILENT, restart: `./tools/sim restart --fast --headless`.

**Everything is very slow.** Check what else the NUC is doing:

```bash
uptime
ps -eo pcpu,args --sort=-pcpu | head -8
```

The NUC is an Intel i7-7567U: **two physical cores, four threads**. Measured from /proc
over twelve seconds during a live run on 2026-09-08, with the viewer attached:

| | CPU |
|---|---|
| `gz sim` | 113% |
| `mission` | 42% |
| `grasp` (waiting its turn) | 39% |
| `deliver` (waiting its turn) | 34% |
| `perception` | 26% |
| `approach` | 24% |
| the rest | ~34% |
| **total** | **312% of the 400% there is** |

Load average was 6.6. When more is asked than there are threads to give, Gazebo gets less
than it wants and the real-time factor falls with it -- 0.05 to 0.18 in that state against
0.42 on an idle machine.

Two of those are avoidable. The viewer is worth about half the speed on its own (0.248
headless against 0.122 attached, measured back to back), and `grasp` and `deliver` between
them burn most of a core while doing nothing but waiting for their phase, which is a fault
in this solution rather than a limit of the machine.

`ps` shows a truncated command and shifting columns; `gz sim` in particular reports itself
as **ruby**, because Gazebo's launcher is a Ruby script. A careless reading of `top` here
turned one Gazebo into four mystery processes.

---

## A view that needs no GPU at all

If the Gazebo window will not stay up, this serves the robot's own camera feeds and the
detector's annotations to a browser, rendered by the server through EGL — the half that
does not have the OpenGL problem:

```bash
./tools/in-sim liveview.py
```

Then open **http://localhost:8080** in a browser on the NUC. It is not a 3D view of the
arena, but it shows what the robot can actually see, which is usually the question.

---

## Recording the competition video

The rules for D2 are specific, and this ordering is what they ask for: **launch
`simulation.launch.py` first, then `solution.launch.py`**, unedited, no cuts, not sped
up, team name on screen before the trial starts, and a timer running.

So for the video, do not use `sim restart` or `run-once.sh` — they wrap those launches
and the wrapping will not be visible. Run them raw, with the window, in two terminals:

```bash
docker exec -it erc_sim /entrypoint.sh bash -c \
  'source /opt/erc_ws/install/setup.bash && ros2 launch erc_bringup simulation.launch.py'
```

```bash
docker exec -it erc_sim /entrypoint.sh bash -c \
  'source /opt/erc_ws/install/setup.bash && \
   ros2 launch avaa_solution solution.launch.py shelf_column_number:=3 book_colour:=red'
```

Two things to plan for. The GUI-at-startup failure above is the reason the first command
sometimes produces a simulator with no controllers — if that happens, stop and start
again before beginning the recording, and check `sim status` shows both sensors
streaming. And the video has a five-minute limit against a mission that takes 157
simulated seconds: at the RTF the NUC manages with a window open you will not fit, so the
recording needs the machine quiet and nothing else competing for its cores.

---

## Why the remote desktop session

Over SSH there is no `DISPLAY`, so `sim gui` has nothing to draw on and falls back to
guessing. Inside the desktop session `DISPLAY` is `:10`, which is the X server xrdp
actually runs, and `sim gui` picks it up. This was worth a commit of its own: the
container was created over SSH, compose baked in `DISPLAY=:0` from its default, and the
viewer spent a session drawing on a display that has never existed on this machine.
