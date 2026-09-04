# Perception — findings and approach

Measured against the live simulation on 2026-08-26. Everything here is empirical; nothing is
assumed from the PDFs. Feeds directly into report section 2 (Perception), which requires
**quantitative accuracy across a number of trials**.

Test images: `reference/perception/`

---

## 1. Colour separation is trivially clean

Hue histogram of all saturated pixels (`S > 100, V > 60`) in a shelf-facing frame:

| Colour | Hue (OpenCV, 0–179) | Median S | Median V |
|---|---|---|---|
| Red | 0–10, wrapping to 170–180 | 255 | 201 |
| Yellow | 30–40 | 255 | 201 |
| Green | 60–70 | 122 | 159 |
| Blue | 120–130 | 255 | 201 |

Four isolated clusters with wide empty gaps between them. There is **no colour ambiguity** —
simple HSV thresholding is sufficient and a learned classifier would be wasted effort. This
matters for Phase 2: the real robot is an i5 / 16 GB with no stated GPU, and the same code has
to run there.

Ranges in use (widened slightly for shading and viewing angle):

```python
"red":    [((0, 90, 60), (12, 255, 255)), ((168, 90, 60), (179, 255, 255))],
"yellow": [((20, 90, 60), (42, 255, 255))],
"green":  [((48, 60, 60), (85, 255, 255))],
"blue":   [((105, 90, 60), (135, 255, 255))],
```

> Green needs a lower saturation floor (60 rather than 90) — the green books measure S≈122,
> markedly less saturated than the other three at S=255.

## 2. ⚠️ Two objects share a hue with the books

This is the real perception problem, and it is not visible from the PDFs:

| Confounder | Shares hue with | Why it matters |
|---|---|---|
| **The collection bin** | red books | Large red blob, always in view from the start zone |
| **The start zone** | green books | Large green floor patch, in view whenever facing the shelves |

Colour alone therefore cannot identify a book. Shape does separate them cleanly:

| Filter | Value | Rejects |
|---|---|---|
| `area` | 30 – 3000 px | the bin (measured 6763 px) |
| `aspect` (h/w) | ≥ 1.2 | start zone patches (measured 0.54, 0.56) |
| `fill` (contour/bbox) | ≥ 0.55 | irregular blobs; books are solid rectangles |

Books are 25 × 16 × 3 cm shelved spine-out, so they always present an upright face. Measured
book detections: 6–8 px wide, 14–26 px tall, aspect 1.75–4.33, fill 0.78–0.83 — a wide margin
from both confounders.

## 3. Detector result on the first test frame

**16 of 16 books detected, 0 false positives.** Four columns visible × four books each, one of
each colour per column, exactly as specified.

```
ACCEPTED 16 books
  yellow ( 10,159)  7x25  aspect=3.57 fill=0.82
  blue   ( 13,192)  7x25  aspect=3.57 fill=0.82
  ...
REJECTED 3
  red    136x66  -> area 6763 > 3000        (the bin)
  green   67x36  -> aspect 0.54 < 1.2       (start zone)
  green   70x39  -> aspect 0.56 < 1.2       (start zone)
```

> Single frame, single viewpoint. Not yet an accuracy figure — the report needs this across
> many trials and viewing angles. See "Open work" below.

## 4. Row numbering — ground truth

The shelf has 6 rows; only the middle 4 are stocked. Gazebo names them `row_2` … `row_5`, and
their measured heights are:

| World name | Z (m) |
|---|---|
| `row_2` | 1.577 |
| `row_3` | 1.247 |
| `row_4` | 0.917 |
| `row_5` | 0.587 |

**Rows count top-down, 0.33 m apart.** Extrapolating, the empty `row_1` sits at 1.907 m and
`row_6` at 0.257 m, consistent with a 210 cm unit.

The competition numbers rows **1–4**. World rows 2–5 are the stocked ones, so:

```
competition row 1  =  world row_2  =  TOP stocked shelf     (Z 1.577)
competition row 4  =  world row_5  =  BOTTOM stocked shelf  (Z 0.587)
```

> ⚠️ **Inference, not confirmed.** It follows from the world's own numbering running top-down,
> which is strong evidence, but the mapping from competition 1–4 onto world 2–5 is not stated
> anywhere. Getting it backwards costs the row identification points (+3) and would send the
> arm to the wrong shelf. **Worth confirming with the organizers.**

## 5. Randomization confirmed empirically

Both randomizations described in the rules are real and were observed directly:

- **Column markers.** One load showed `3, 5, 1` left-to-right; a later load showed `2, 1, 5, 3`.
- **Colour → row.** `book_col_1` held yellow/red/green/blue on rows 2/3/4/5 in one load, and
  red/green/blue/yellow in another.
- **Horizontal position.** Column 1's books measured Y = 1.78, 2.18, 2.06, 2.14 — scattered
  within the row, matching the stated 25–75% of row width.

All books sit at X = 2.90 (shelf front face; the unit is centred at X = 3.0, 30 cm deep).

**Nothing spatial can be hardcoded.** Confirmed, not assumed.

## 6. Implemented in the package

| File | Role |
|---|---|
| `avaa_solution/vision/book_detector.py` | Pure OpenCV detector — no ROS import, runs on saved frames |
| `avaa_solution/perception_node.py` | ROS wrapper: subscribes, publishes, saves annotated images |
| `test/test_book_detector.py` | 14 unit tests on synthetic frames, no simulator needed |

Topics:

| Topic | Type | Contents |
|---|---|---|
| `/avaa/perception/books` | `vision_msgs/Detection2DArray` | every book; colour in `class_id`, column index in `id` |
| `/avaa/perception/target_row` | `std_msgs/Int32` | row of the target book, when resolvable |

Design decisions worth keeping:

- **The perception node does not publish to `/erc/shelf_row_identification`.** The mission
  node owns the scoring topics, so there is exactly one place responsible for what reaches
  the committee.
- **`BEST_EFFORT` QoS on the camera subscription.** The camera publishes best-effort; a
  reliable subscriber receives nothing at all. Silent failure if you get this wrong.
- **`row_of()` returns `None` rather than guessing** when fewer than four books are visible
  in the column — a missing top book shifts every row by one.
- **Timestamps are burned into the image**, not just the filename, since a filename alone is
  trivially altered after the fact and the rules require judges to be able to verify capture
  time.

### Verified live

```
[avaa_perception] perception up — target colour red
[avaa_perception] target red book is on row 4
erc_images/row_4_red_2026-08-26T05-32-31.143.png
```

Cross-checked against the frame: leftmost column reads green/yellow/blue/red top to bottom,
so red is row 4. Correct.

### ⚠️ Where `erc_images/` can live

Currently `/opt/erc_ws/src/avaa_solution/erc_images` (a node parameter).

The stock `docker-compose.yml` bind-mounts **only** `../src` to `/opt/erc_ws/src`. Nothing
above that is writable from inside the container, so a top-level `erc_images/` at the repo
root **cannot** be written by the solution during a trial under the stock setup. The chosen
path is the deepest one that still lands inside the git repository on the host.

Files land owned by `root`, since the container runs as root — a minor annoyance for `git`,
not a blocker. Combined with `NOTES.md` §12.5 (the doc writes `/erc_images/` with a leading
slash but calls it a folder "in the team's repository"), **this is worth one question to the
organisers.**

## 7. Tooling built

| Script | Purpose |
|---|---|
| `sweep.py` | Rotate the base and capture frames, to collect test data |
| `turn_capture.py` | Turn by a relative angle using **odometry feedback**, then capture |
| `analyse_colours.py` | Hue-cluster histogram of a frame |
| `detect_books.py` | Colour + shape book detector, with reject reasons |

> `turn_capture.py` is closed-loop for a reason: at a variable real-time factor of 0.35–0.53,
> open-loop "publish 0.5 rad/s for 4 s" does not produce a repeatable angle. Commanding
> +2.503 rad achieved +2.483 rad.

---

## 8. Column marker digits — solved

### The templates ship with the simulator

The marker plates are 30 × 30 cm at **Z = 2.26 m**, X = 2.755, spread across
`Y = (2 − col) × COLUMN_WIDTH`. The digit is a **PNG texture** substituted into
`erc_number_marker.sdf` at spawn:

```python
shelf_number_markers = list(range(1, NUM_COLUMNS + 1))
random.shuffle(shelf_number_markers)          # digit -> column, reshuffled per load
... '-name', f'number_marker_col_{col+1}'      # the name encodes the COLUMN, not the digit
```

`erc_description/models/number_marker/textures/` contains `0.png`–`9.png`. **Matching
against the simulator's own artwork** removes a whole class of error and needs no training
data — of which the competition provides none.

### Method

Binary-mask **IoU after letterboxed normalisation**. Letterboxing rather than stretching
preserves width-to-height inside the mask, which is what separates a `1` from the rest: its
texture is 88 px wide against 110–117 for the others.

`MIN_SCORE = 0.35`, calibrated rather than guessed. On a frame where all four visible digits
read correctly, IoU ranged **0.43–0.75** — the *worst* being the **nearest** plate, which is
viewed most obliquely and so is the most perspective-skewed. A threshold of 0.45 would have
rejected a correct read.

**No margin threshold.** A correctly-read `5` measured a margin of only 0.042 over `3` at
15 px. Rejecting on that would have discarded a correct answer.

### Measured accuracy

Nine viewpoints across a yaw sweep: **every visible digit read correctly at every pose**,
IoU 0.41–0.72.

```
pose 0: [1, 5, 3]        scores=[0.49, 0.62, 0.67]
pose 4: [2, 1, 5, 3]     scores=[0.41, 0.56, 0.67, 0.70]
pose 8: [2, 1, 5, 3]     scores=[0.44, 0.54, 0.66, 0.72]
```

The `[1,5,3]` poses are not misreads — the `2` plate was outside the frame at those angles.

> **The distinct-digit constraint has not yet changed an answer.** Independent reading was
> already correct in all nine poses. It is insurance against low-resolution ambiguity, not a
> current fix. Reported honestly because the report should not overclaim it.

### ⚠️ Gap clustering was wrong; markers define the columns

`group_into_columns` clusters books on horizontal gaps, which needs a threshold, and that
threshold is scale-dependent. From an oblique angle **two adjacent columns sat closer
together in the image than books within a single column do at close range**, and the
detector merged them — reporting "column 3 shows 8 of 4 books".

`group_by_anchors` fixes it: each marker sits above exactly one column, so the marker
positions *are* the column positions, and the column index equals the marker index. Gap
clustering is kept only as a fallback for when no marker is in view — where no column can be
identified anyway.

### End to end

```
[avaa_perception] marker 5 is column index 2 (4 column(s) in view)
[avaa_perception] target blue book is on row 4
erc_images/column_5_*.png    box around the marker and its four books
erc_images/row_4_blue_*.png  box around the target book
```

---

## 9. 3D book localisation — x and y are good, z is not

`avaa_solution/vision/depth_locator.py` turns a detected book's pixel box into a 3D point:
sample the depth over a shrunk central patch, deproject through the camera intrinsics,
transform to `base_link` via TF. Published on `/avaa/perception/target_book_point`.

The RGB and depth streams share intrinsics exactly (`fx = fy = 337.2096`, `cx = 320`,
`cy = 180`, both 640×360), so a box found in colour indexes the depth image directly.
Depth arrives as `32FC1` in metres.

### Measured against ground truth

Target `book_col_4_row_4_blue`, true world pose **(2.900, −1.237, 0.917)**. Our estimate,
converted to world: **(2.865, −1.222, 1.025)**.

| Axis | Error | Verdict |
|---|---|---|
| x | −0.035 m | ✅ expected — we see the book's front face, and it is 0.03 m deep |
| y | +0.015 m | ✅ good |
| z | **+0.108 m** | ❌ too large to grasp with |

Books are 0.25 m tall and rows are 0.33 m apart, so 0.108 m of vertical error is a third of
the gap between shelves.

### ⚠️ The vertical bias is systematic and not yet explained

It is not specific to books. Deprojecting the column markers, whose plates sit at a known
world z of 2.26, gives **2.42–2.48** — high by a similar margin.

Ruled out so far:

- **Occlusion of the book's lower half.** The detected box is 32 px tall where the geometry
  predicts 29.9 px, so the whole book is visible.
- **Model origin not at the book's centre.** The SDF visual is a `0.25 × 0.03 × 0.16` box at
  pose `0 0 0`, so the origin *is* the centre.
- **Using colour pixels with the depth frame.** The two optical frames differ by
  (−0.015, 0, 0) m with no rotation — 15 mm laterally, nothing vertical.

Still open: whether the error scales with angle from the principal point. The marker sits
142 px off centre and is out by ~0.19 m; the book sits 30 px off centre and is out by
0.108 m. Those are not proportional, so a single scale error does not explain it either.

### The way around it: take z from the row, not from depth

This does not need solving to proceed. **Row identification is already reliable** — it named
row 3 for a book the simulator calls `row_4`, which is the correct mapping — and the row
determines the height. So:

| Quantity | Source |
|---|---|
| Height (z) | the **identified row** |
| Distance (x) and lateral offset (y) | **depth**, accurate to 15–35 mm |

That uses each measurement where it is trustworthy, and sidesteps the bias entirely. The
row → height calibration should be measured once in simulation rather than taken from the
world file, so it stays honest about what the robot can actually observe.

## Open work

1. **Accuracy across trials, with ground truth.** Nine viewpoints in one world load is not the
   same as many loads. Automating this is blocked on ground truth: `gz model` does not expose
   the marker's texture, the model name encodes only the column, and the spawn arguments are
   not in the launch logs. Options are hand-labelling a set of loads, or reading each marker
   from close range where the digit is unambiguous and using that as truth for the harder
   long-range reads.
2. **Confirm row numbering direction** with the organisers (section 4).
3. **Partial column views.** `row_of` returns `None` unless all four books are visible. A
   spacing-based estimator could recover the row from two or three, since rows are evenly
   spaced 0.33 m apart.
4. **Perspective correction.** The nearest plate scores worst because it is most skewed.
   Detecting the plate quad and warping it flat before matching would raise scores where they
   are currently lowest.
