# Vision pipeline — AprilTag localization for the arm

Handoff document. Read this instead of re-deriving the design.

## What it does

A statically-mounted USB camera locates a tagged cube in the arm's `base_assy`
frame so MoveIt can pick it.

**The camera's extrinsic is NOT stored.** It is re-solved from the base tags in
every capture, from the same image that sees the cube:

```
base tags   -> T_base<-cam     (where the camera is, right now)
cube tags   -> T_cam<-cube     (where the cube is, relative to the camera)
compose     -> T_base<-cube    the camera drops out
```

Two consequences worth keeping: a bumped camera cannot silently corrupt results
(the geometry changes in the very image being used), and common-mode errors
partly cancel because the answer depends on the *relative* geometry of base tags
and cube within one frame.

**Intrinsics are the only stored calibration**, so they are the dominant
systematic. A 1% focal error is a 1% depth error at every range.

## Files

| File | Role |
|---|---|
| `tag_map.py` | Map loading/validation, tag corner geometry. Pure. |
| `camera_extrinsics.py` | The PnP solver, pose averaging, leave-one-out. Pure. |
| `scene_solve.py` | Composes the two solves into a cube pose. Pure. |
| `apriltag_detect.py` | Detector wrapper + filtering. |
| `camera_capture.py` | The ONLY place a camera is opened (except `tune_camera`). |
| `locate_cube.py` | CLI: capture -> solve -> report -> YAML + overlay. |
| `cube_pick_target.py` | The MoveIt seam. Returns pick waypoints. |
| `camera_tf_publisher.py` | ROS2 node broadcasting camera/cube/grasp TF. |
| `moveit_pick_cube.py` | **The pick.** One capture → TF + scene + MoveIt motion. |
| ↳ on failure | writes `pick_cube.failed.png` + shows it; shares `locate_cube.annotate_failure`/`diagnose`. |
| `pick_cube.launch.py` | Runs it with the MoveIt config params attached. |
| `calibrate_intrinsics.py` | ChArUco intrinsics, guided capture. |
| `calibrate_camera_extrinsics.py` | **Validation only** — nothing at runtime reads its output. |
| `make_apriltags.py` / `make_cube_net.py` / `make_charuco_board.py` | Printables (vector PDF). |
| `pdf_sheet.py` | Minimal dependency-free vector PDF writer. |
| `tune_camera.py` | Sweeps camera controls, scored by tag detections. |

Data: `tags_base_assy.yaml` (base map), `cube_tags.yaml` (cube map),
`camera_intrinsics.yaml`, `constants.py` (intrinsics + `CAMERA_SETTINGS`).

The pure modules import only numpy/cv2/scipy/yaml — no camera, no ROS — so a
ROS node and a CLI share them unchanged.

## Frames

- **`base_assy`**: X=0,Y=0 is the turret rotation axis; Z=0 is the underside of
  the base plate. Metres and radians everywhere.
- **Tag frame**: origin at tag centre, +X right as printed, +Y up as printed,
  +Z out of the face. `rpy` is extrinsic XYZ, same as URDF `<origin rpy>`.
- **The `xyz` in a map is the tag CENTRE** (centroid of the four corners), and
  `size` is the edge of the **black border square**, not the quiet zone.
- Base tag ids 0–9, cube ids 10–14. `scene_solve.check_disjoint()` enforces it.

## Hard-won findings — do not rediscover these

1. **`cv2.aruco` renders `DICT_APRILTAG_*` rotated 180°** vs the apriltag
   library's canonical orientation. True for both 36h11 and 16h5. The
   generators correct for it. A naive synthetic test will appear to prove
   `TAG_CORNER_OFFSETS_UNIT` wrong by a 2-index roll; it isn't.
2. **`dt_apriltags` registers only ONE family.** Its docstring claims space-
   separated families work; the implementation is an `if/elif` chain
   (`apriltags.py:266`) and `tag16h5` wins over `tag36h11`. Mixing families in
   one scene needs two detectors.
3. **`SOLVEPNP_IPPE_SQUARE` assumes its own corner order** (TL,TR,BR,BL, +Y up),
   not ours (BL,BR,TR,TL). Feeding it our points gives 753 mm / 158° error while
   looking plausible. Use `SOLVEPNP_IPPE` or `SQPNP`.
4. **Never call `flags=0` (ITERATIVE) cold** — DLT init is degenerate for
   coplanar points. Use SQPNP/IPPE then refine with VVS.
5. **Leave-one-out: read `remaining_rms`, not `held_rms`.** One bad tag inflates
   every fit containing it, so all held-out errors rise together. The culprit is
   the tag whose *removal* makes the rest snap clean. `suspect_tag()` does this.
6. **A common z offset on all base tags is invisible to reprojection RMS.** The
   tags stay consistent with each other and the answer is silently wrong. Only
   physical measurement or the `--verify` triad overlay catches it.
7. **PNG DPI metadata is advisory** and most print paths ignore it. Print the
   **PDF** (vector, absolute physical size) at 100% / actual size.
8. **`cv2.VideoCapture` is a C type and rejects new attributes** — hence the
   `camera_capture.Camera` wrapper. Python test doubles accept attributes
   happily, so this class of bug hides from fakes.
9. **V4L2 silently CLAMPS out-of-range control values** to the nearest legal one
   (often the worst setting) and reports success. `open_camera` now verifies via
   readback and warns.
10. **Tag family does not affect pose accuracy at equal physical size** —
    measured identical (0.85 mm, 0.079°) for 16h5 vs 36h11. 16h5 only wins on
    minimum readable size (past ~3.6 m for a 60 mm tag) and costs false
    positives (164 past the margin filter vs 0, over 60 tag-free frames);
    `hamming == 0` catches them. **36h11 is the right default.**
11. **Coplanar base tags are fine if widely spread.** Three tags at 400×375 mm
    gave 0.26–1.5 mm with ambiguity ratios of 110–334 (threshold 3). A raised
    vertical tag can make things *worse* at high camera elevation because it
    foreshortens.
12. **Calibration coverage beats RMS.** A centre-biased ChArUco session scored
    *better* RMS (0.186 vs 0.265) but 10× worse focal error. Judge coverage and
    tilt, not RMS alone.
13. **Convert the RMS gate to millimetres before believing it.** One pixel is
    range/focal: at 0.6–0.85 m with f = 1007 px that is 0.6–0.83 mm. The
    original 1.5 px gate therefore demanded ~1 mm of agreement — 3–4× tighter
    than the rig's ~4 mm need — and failed base maps that were provably fine
    (calipers 60.0 mm, every tag reprojecting to a clean square, positions
    consistent to ~1.5 mm). `max_rms_px` is now **4.0** in `scene_solve`,
    `cube_pick_target`, `locate_cube`, `camera_tf_publisher` and
    `moveit_pick_cube`; `calibrate_camera_extrinsics` stays at 1.5 because it
    only validates. **4.0 px ≈ 3.3 mm at the far tag and the 4 mm budget is
    4.8 px — there is almost no headroom left.** Past that the gate stops
    enforcing anything. If it needs raising again, fix the image instead: more
    pixels on the tag (closer camera, larger print) or less clipping lets the
    gate come back down. Re-derive it whenever the geometry changes.
14. **A one-sided bias across every cross-check distance means scale, not a
    moved tag.** A misplaced tag pushes some pairs out and pulls others in.
    All-positive (or all-negative) diffs point at `default_size` or focal
    length. Distinguish them: tag size has a clean interior minimum when swept
    against fixed map centres, whereas focal length just improves monotonically
    — that direction is degenerate and is not evidence.
15. **The distance cross-check is single-tag PnP and is NOISY.** On ~50 px
    coplanar squares it scattered ±5 mm when the 12-point joint fit put the map
    at 1.5 mm. That is why `diagnose()` only warns past 8 mm. Read it for
    *direction and sign*, not magnitude.
16. **An arm that does not MOVE poisons every later waypoint.**
    `set_start_state_to_current_state()` reads the real arm, so if a segment
    does not execute, the next one plans from the stale pose. Pilz LIN
    interpolates orientation as well as position, so a descent planned from a
    start whose tool is ~180° away fails on its FIRST interpolation step — and
    the error surfaces at the descent, far from the actual cause. Two triggers,
    both now caught explicitly:
    - `--plan-only` never executes by design. It cannot validate a sequence,
      only individual poses; straight-line segments are planned with OMPL there
      and it says so.
    - **`MoveItCpp::execute()` returns an `ExecutionStatus` that was being
      discarded.** With no active controllers it logs "No active controllers
      configured for group" and returns ABORTED, which looked like success.
      Both task scripts now check it, and `moveit_pick_cube` additionally
      compares the arm's joints against the trajectory's final point
      (`--arrival-tol-deg`, default 5°) to catch a controller that accepts a
      goal while the motors are off.

    The descent geometry itself is fine and was measured: both endpoints
    reachable tool-down, manipulability ~0.043 (nowhere near singular), 47° of
    joint-limit margin, peak joint rate 2.4 rad/m, and all 4 IK branches can
    follow it. If the descent fails on a run that genuinely moves, suspect
    something new — not the geometry.
17. **`ee` is NOT the fingertip — 30.5 mm of gripper hangs below it.** The
    `ee` frame sits 62.8 mm along `6th_wrist_joint`; `gripper_pincher_2.stl`
    reaches 93.2 mm. Sideways this never mattered, which is why pick-and-pour
    never hit it. Pointing the tool DOWN it matters completely: aiming the ee
    frame at a cube centre 20.5 mm above the table puts the pinchers 10 mm
    *under* the table and MoveIt refuses with GOAL_IN_COLLISION. Raising the
    commanded ee by the tool reach puts the PINCH POINT on the cube centre,
    where it belonged, and clears the table by 30 mm (`--tool-reach`, default
    0.0305). Any new top-down tooling needs this number re-measured.
18. **Read the planner's error code.** `plan()` returns a response carrying
    `.error_code`, `.planner_id` and `.planning_time`; logging a bare
    "Planning FAILED" throws away the one fact that identifies the cause and
    costs hours of guessing at geometry that was fine. Both task scripts now
    print the code name plus a hint per code.
19. **Scaling an isolated object's map uniformly is invisible to reprojection.**
    Scaling `cube_tags.yaml` (sizes *and* face offsets) ±3 % left the fit flat
    at 1.03 px — the camera just moves. So RMS can never confirm the cube net
    printed at the right scale; only calipers can. Base tags are different:
    there the size changes while the mapped centres stay put, which *is*
    observable.

## This camera (Rocketfish HD Webcam)

- **Manual exposure is a no-op in MJPG; it works in YUYV.** Firmware quirk,
  verified: brightness/contrast/gamma respond in both, exposure only in YUYV.
- MJPG gives 30 fps at 1280×720; YUYV gives 10 fps. Fine for burst-then-solve.
- **Field of view is identical between formats** (scale 1.00025), so intrinsics
  transfer without recalibration.
- `contrast` min is 60, default 136. Values under 60 clamp to the worst setting.
- Configure via `constants.CAMERA_SETTINGS`; CLI flags default to `None` so they
  only override when passed. `open_camera` prints the effective settings.

## Commands

```bash
# Printables
./venv/bin/python make_apriltags.py --family tag36h11 --ids 0,1,2   # base tags
./venv/bin/python make_cube_net.py --cube 0.060                     # cube wrap
./venv/bin/python make_charuco_board.py                             # calib board

# Calibration
./venv/bin/python calibrate_intrinsics.py --save-dir ./cal_pics     # intrinsics
./venv/bin/python calibrate_camera_extrinsics.py --corner-check --display  # GATE
./venv/bin/python calibrate_camera_extrinsics.py --display --verify # validate map

# Runtime
./venv/bin/python locate_cube.py --display --verify
./venv/bin/python tune_camera.py --sweep exposure --live
```

`locate_cube.py --image FILE` solves a saved still — useful for debugging
without the rig. On failure it prints a DIAGNOSIS block (per-tag RMS,
leave-one-out, and a camera-measured vs map distance cross-check that isolates a
mis-measured coordinate from bad image quality) and writes `*.failed.png`.

## State

Working: intrinsics (ChArUco, 0.388 px RMS at 1280×720); base tag map with 3
coplanar tags detecting reliably; cube net printed and verified in simulation
(folded net reproduces `cube_tags.yaml` to 0.028 mm).

Pending:
- Notes in `tags_base_assy.yaml` still say PLACEHOLDER. `default_size` is
  confirmed 60.0 mm by calipers, so it is the `xyz` notes that are stale.
- `moveit_pick_cube.py` has NOT been run against the real arm yet. Its vision
  half is exercised; the MoveIt half (collision objects via `moveit_py`,
  `DisplayTrajectory`, whether a grasp down at the table plane plans at all
  under the fixed orientation) is unverified. Start with `plan_only:=true`.
- Grasp **orientation** now defaults to tool-straight-down (`--orientation
  down`), derived from the tool tip lying along `6th_wrist_joint`'s +Z, so
  "down" is `Rz(azimuth) @ Ry(pi)`. The inherited pick-and-pour pose holds the
  tool *horizontal* — a side grasp for a bottle — and is kept as
  `--orientation fixed`. `--orientation vision` (from `cube_tags.yaml`'s
  `grasp:` block) is still untuned against the gripper.
- The cube sits near z ≈ 0.02 while every tuned pick-and-pour waypoint is at
  z = 0.175. Reaching that low is still the most likely planning failure, and
  changing the orientation changes which poses are reachable — re-run
  `plan_only:=true` after any orientation change.

## Integration contract

`moveit_pick_and_pour.py` uses `BASE_FRAME="base_assy"`, `EE_FRAME="ee"`, metre
waypoints, and one hardcoded orientation composed with `_EE_FRAME_OFFSET_RPY`.
`cube_pick_target.get_pick_target()` returns `.grasp/.approach/.lift` xyz plus
`.quat`, and raises `NoTargetError` rather than returning a doubtful pose.
Capture before the arm moves — the static camera gets occluded once it does.

`moveit_pick_cube.py` is that wiring, as a task of its own rather than an edit
to the working pick-and-pour: it subclasses `PickAndPour` only to inherit the
tuned `move_to_pose`/`gripper`/plan helpers. It publishes the frames from the
*same* solve it moves on, so RViz cannot disagree with the arm.

**Only one process may open the camera.** `moveit_pick_cube.py` and
`camera_tf_publisher.py` both do; run one or the other, never both.
