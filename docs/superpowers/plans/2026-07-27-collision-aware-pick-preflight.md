# Collision-Aware Pick Preflight Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan.

**Goal:** Prevent the executor from driving to, or beginning an arm pick at, a base pose whose final floor-clamped pre-pick/pick states collide with the modeled printer bed.

**Architecture:** Keep the full base grid as an IK-only filter, collect a small ordered shortlist, and collision-check that shortlist serially against a temporary predicted bed object held under one MoveIt planning-scene write lock. After the requested drive and any lidar translation correction, rebuild the actual bed object and re-run IK plus collision checking before arm motion. Put ROS-independent candidate selection, drive bookkeeping, and orientation-attempt sequencing in a small helper module so the safety decisions are testable on macOS without ROS.

**Tech Stack:** Python 3, ROS 2 Humble, MoveItPy, `unittest`, CMake/ament.

---

## Preconditions and invariants

- Work only in the `codex/collision-aware-pick-preflight` worktree.
- Follow red-green-refactor for every production behavior below.
- Never call the planning scene from inside the full grid loop.
- Collision-query errors fail closed for protected picks.
- A temporary preflight slab is added and removed synchronously through
  `PlanningSceneMonitor.read_write()` and
  `PlanningScene.apply_collision_object()`. Both APIs are present in the
  MoveIt 2 Humble bindings.
- Placement searches and runs with `bed_collision:=false` preserve their
  current IK-only behavior.
- The `printer_bed` object is not left in the local planning scene after
  hypothetical preflight.
- No arm step (`01_home` onward) can run unless post-drive validation has
  passed for the current target coordinates and current bed estimate.

## Task 1: Add pure safety-decision and drive-bookkeeping helpers

**Files:**

- Create: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/pick_preflight.py`
- Create: `src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py`
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/CMakeLists.txt`

### Step 1: Write failing tests

Add `unittest` cases for the wished-for pure API:

```python
from pick_preflight import (
    DriveDelta,
    IKCandidate,
    apply_drive_delta_xy,
    choose_collision_validated_candidates,
    run_orientation_attempts,
)


class PickPreflightTests(unittest.TestCase):
    def test_lidar_translation_is_part_of_total_drive(self):
        delta = DriveDelta(0.24, 0.01, 0.013, -0.004)
        self.assertEqual(delta.total, (0.253, 0.006))
        self.assertEqual(apply_drive_delta_xy(0.70, -0.02, delta), (0.447, -0.026))

    def test_colliding_candidate_is_rejected_for_later_orientation(self):
        first = IKCandidate(0.24, 0.0, 0, ("pre-a", "pick-a"), (0.255, 0.195))
        second = IKCandidate(0.24, 0.0, 2, ("pre-b", "pick-b"), (0.255, 0.195))
        accepted, rejected = choose_collision_validated_candidates(
            [first, second], lambda candidate: candidate.orientation_index == 2
        )
        self.assertEqual(accepted, [second])
        self.assertEqual(rejected, 1)

    def test_later_base_offset_is_kept_when_first_offset_collides(self):
        # All orientations at dx=.24 fail, dx=.27 survives.
        candidates = [
            IKCandidate(0.24, 0.0, 0, ("a0", "a1"), (0.255, 0.195)),
            IKCandidate(0.24, 0.0, 2, ("b0", "b1"), (0.255, 0.195)),
            IKCandidate(0.27, 0.0, 1, ("c0", "c1"), (0.255, 0.195)),
        ]
        accepted, rejected = choose_collision_validated_candidates(
            candidates, lambda candidate: candidate.dx == 0.27
        )
        self.assertEqual(accepted, [candidates[2]])
        self.assertEqual(rejected, 2)

    def test_scene_exception_fails_candidate_closed(self):
        # Validator raises; candidate is rejected and the next one is evaluated.
        candidates = [
            IKCandidate(0.24, 0.0, 0, ("a0", "a1"), (0.255, 0.195)),
            IKCandidate(0.27, 0.0, 1, ("b0", "b1"), (0.255, 0.195)),
        ]
        calls = []

        def validate(candidate):
            calls.append(candidate.dx)
            if candidate.dx == 0.24:
                raise RuntimeError("scene unavailable")
            return True

        accepted, rejected = choose_collision_validated_candidates(
            candidates, validate
        )
        self.assertEqual(accepted, [candidates[1]])
        self.assertEqual(rejected, 1)
        self.assertEqual(calls, [0.24, 0.27])

    def test_planning_failure_after_ik_success_tries_next_orientation(self):
        # First attempt returns False; second returns True and is selected.
        attempts = []
        winner = run_orientation_attempts(
            [0, 1, 2], lambda index: attempts.append(index) or index == 1
        )
        self.assertEqual(winner, 1)
        self.assertEqual(attempts, [0, 1])
```

The actual test file must contain complete assertions, not ellipses.

### Step 2: Verify red

Run:

```bash
python3 -m unittest discover \
  -s src/yahboom_rosmaster/gemini_pick_place_executor/test \
  -p 'test_pick_preflight.py' -v
```

Expected: import failure because `pick_preflight.py` does not exist.

### Step 3: Implement the minimal pure module

Implement:

```python
@dataclass(frozen=True)
class DriveDelta:
    requested_dx: float
    requested_dy: float
    correction_dx: float = 0.0
    correction_dy: float = 0.0

    @property
    def total(self):
        return (
            self.requested_dx + self.correction_dx,
            self.requested_dy + self.correction_dy,
        )


@dataclass(frozen=True)
class IKCandidate:
    dx: float
    dy: float
    orientation_index: int
    joint_solutions: tuple
    fingertip_zs: tuple


def apply_drive_delta_xy(x, y, delta):
    total_dx, total_dy = delta.total
    return float(x) - total_dx, float(y) - total_dy


def choose_collision_validated_candidates(candidates, validator):
    accepted = []
    rejected = 0
    for candidate in candidates:
        try:
            is_safe = bool(validator(candidate))
        except Exception:
            is_safe = False
        if is_safe:
            accepted.append(candidate)
        else:
            rejected += 1
    return accepted, rejected


def run_orientation_attempts(items, attempt):
    for item in items:
        if attempt(item):
            return item
    return None
```

Keep the helper independent of ROS and MoveIt imports.

### Step 4: Verify green

Run the unittest command from Step 2.

Expected: all helper tests pass.

### Step 5: Install the helper

Add `scripts/pick_preflight.py` to the package install rules so the installed
executor can import it from its own executable directory.

### Step 6: Commit

```bash
git add \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/pick_preflight.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/CMakeLists.txt
git commit -m "test: add pick preflight safety helpers"
```

## Task 2: Account for lidar correction in all tracked points

**Files:**

- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py`
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py`

### Step 1: Add failing bookkeeping tests

Add cases proving:

- a no-correction `DriveDelta` returns exactly the requested motion;
- a lidar correction is added once, not omitted or double-counted;
- the same total delta transforms both target and destination coordinates.

Run the helper test command and observe the new tests fail until the API has the
necessary convenience behavior.

### Step 2: Return the commanded lidar translation

Change `_lidar_audit_drive` to return `(0.0, 0.0)` from every path that does not
command a correction. When it commands `(cx, cy)`, return those values only if
`drive_relative_base(cx, cy)` succeeds. If the correction drive reports
failure, log an error and return `None` so the caller aborts rather than
pretending the point frame is known.

Yaw correction remains separate and does not change point translation.

### Step 3: Use `DriveDelta` in drive methods

In `drive_staging` and `drive_to_feasible`:

1. execute the requested displacement;
2. call `_lidar_audit_drive`;
3. abort on a `None` correction result;
4. create `DriveDelta(requested_dx, requested_dy, correction_dx, correction_dy)`;
5. transform `point.point.x/y` once with `apply_drive_delta_xy`;
6. return the total `(dx, dy)` so all existing callers dead-reckon the
   destination using the complete physical translation.

Update logs to show requested, lidar correction, and total motion.

### Step 4: Verify

Run:

```bash
python3 -m unittest discover \
  -s src/yahboom_rosmaster/gemini_pick_place_executor/test \
  -p 'test_pick_preflight.py' -v
python3 -m py_compile \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/pick_preflight.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py
```

### Step 5: Commit

```bash
git add \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
git commit -m "fix: track lidar correction in scene points"
```

## Task 3: Build an ordered IK shortlist using the final pick height

**Files:**

- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py`
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/launch/executor.launch.py`
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py`

### Step 1: Add failing candidate-selection tests

Add tests that prove:

- candidate ordering is preserved;
- multiple orientations at one offset remain distinct candidates;
- shortlist truncation is deterministic;
- the selected candidate retains both joint solutions and both fingertip
  heights.

Add a pure `ordered_shortlist(candidates, limit)` helper if needed. Observe red,
then implement the minimum helper.

### Step 2: Add the parameter

Declare and forward:

```text
collision_preflight_candidates = 5
```

Clamp it to at least one when read.

### Step 3: Split search from acceptance

Refactor `find_feasible_drive_for_point` so the grid remains unchanged through
IK solving, but:

- change the docstring and logs from “feasible” to “IK-reachable”;
- store each all-lifts IK result as an `IKCandidate`;
- keep searching until the configured number of IK-reachable base offsets is
  reached, retaining every orientation at those offsets;
- do not query the planning scene while collecting candidates;
- do not write `_preferred_orient_idx` or `_search_seed_joints` until a
  collision-validated candidate is selected;
- return an ordered list for protected picks;
- preserve the first-candidate result for unprotected placement/simulation
  paths.

The shortlist should preserve the existing `(dx, dy)` scoring and orientation
order, including later orientations at the same base offset.

### Step 4: Pass the real floor-clamped descent

At every protected-pick call site after `table_z` is known, replace:

```python
self._grasp_descent_nominal(object_height)
```

with:

```python
self._clamp_grasp_descent(target_point, object_height, table_z)
```

This includes the corrected initial pick drive and retry correction drive.

For the pre-table initial non-reperception search, keep it explicitly
kinematic-only and require the corrected, floor-aware search before the pick.
The protected collision preflight is not allowed until `table_z` and a bed
surface are known.

### Step 5: Verify

Run the unittest and `py_compile` commands from Task 2. Also run:

```bash
rg -n "find_feasible_drive.*feasible|feasible base offset" \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py
```

Expected: no user-facing log claims an IK-only result is fully feasible.

### Step 6: Commit

```bash
git add \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/launch/executor.launch.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
git commit -m "refactor: shortlist IK reachable pick offsets"
```

## Task 4: Add serialized predicted-bed collision preflight

**Files:**

- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py`
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py`

### Step 1: Add failing fail-closed decision tests

Cover:

- pre-pick clear plus pick colliding rejects the candidate;
- both states clear accepts the candidate;
- scene mutation failure rejects the candidate;
- scene collision-query failure rejects the candidate;
- a later orientation/base offset can be accepted after rejection.

Use real pure selection code and a small validator double.

### Step 2: Extract bed object construction

Refactor `_publish_printer_bed_collision` into:

```python
def _printer_bed_collision_object(self, wall_x, surface_z, operation):
    obj = CollisionObject()
    obj.id = "printer_bed_preflight"
    obj.operation = operation
    return obj

def _resolve_bed_anchor(self, table_z):
    # returns (wall_x, surface_z, source_description) or None
    return resolved_anchor
```

Both publishing and preflight must use the same dimensions, offset, clearance,
frame, object id, and top-height calculation.

For a candidate relative to the current base frame:

```python
predicted_wall_x = current_wall_x - candidate.dx
```

The slab already spans the full configured lateral workspace; retain its
current `y=0` modeling convention.

### Step 3: Validate states under one write lock

Implement `_candidate_is_collision_free(candidate, wall_x, surface_z, label)`:

1. reconstruct a `RobotState` for each cached joint solution;
2. acquire `psm.read_write()`;
3. apply a temporary `printer_bed_preflight` ADD object;
4. call `scene.is_state_colliding(...)` for pre-pick and pick;
5. in `finally`, apply a REMOVE object before releasing the same lock;
6. return false on collision;
7. catch any mutation/query exception outside the lock, log it, and return
   false.

Use a separate temporary id so an existing real `printer_bed` object is never
destroyed by hypothetical checks.

This direct local mutation makes the exact object immediately visible to the
collision query without topic delays and avoids concurrent planning-scene
access.

### Step 4: Select only collision-validated candidates

For `engage_last_lift=True`, `bed_collision=True`, and a known
`collision_surface_z`:

- resolve the current wall anchor;
- run pure `choose_collision_validated_candidates` on the shortlist;
- accept the first validated candidate;
- cache all validated candidates for bounded post-drive fallback;
- set the preferred orientation and cached joint seeds from the accepted
  candidate;
- if every candidate is rejected or the anchor is unavailable, abort before
  calling `drive_relative_base`.

For unprotected paths, select the first IK-reachable candidate as before.

Add `collision_surface_z` to `drive_to_feasible` and pass the resolved
`table_z`/bed probe from the corrected initial pick and retry calls.

### Step 5: Log safety decisions

For each candidate, include:

- `(dx, dy)`;
- orientation index;
- pre-pick and pick fingertip z;
- predicted slab top;
- `collision-preflight accepted/rejected`.

The chosen log must say `collision-validated base offset`.

### Step 6: Verify

Run helper tests, `py_compile`, and:

```bash
git diff --check
```

### Step 7: Commit

```bash
git add \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
git commit -m "feat: reject bed-colliding pick offsets before drive"
```

## Task 5: Revalidate after the complete physical drive

**Files:**

- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py`
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py`

### Step 1: Add a failing arm-gate test

Add a pure helper such as:

```python
def arm_motion_allowed(post_drive_valid):
    return post_drive_valid is True
```

Test `True`, `False`, and `None`; only literal success may open the arm gate.
Also test a bounded candidate iterator does not repeat an already-attempted
candidate.

### Step 2: Add actual-state validation

After requested drive plus lidar translation:

1. use the point's updated coordinates and the same lifts;
2. resolve the final live/dead-reckoned wall and bed surface;
3. recompute IK for the selected orientation in the current frame;
4. collision-check both reconstructed states against the actual slab using the
   same synchronous planning-scene mechanism;
5. record a one-shot `_pick_preflight_ready` gate containing the validated
   target coordinates, lifts, and slab top.

If the validation fails, keep the arm stowed. If another preflight-validated
candidate exists, convert its original displacement to a delta from the total
physical displacement already applied, drive there, account for its lidar
correction, update the point, and validate again. Bound this to the shortlisted
candidates.

If every post-drive option fails, return `None` and enter the existing
abort/retry path.

### Step 3: Enforce the arm gate

At `_run_pick_phase` entry, verify that `_pick_preflight_ready`:

- exists;
- matches the current target x/y and final pre-pick/pick z values within a
  small numeric tolerance;
- was produced for the current pick attempt.

If not, log a safety error and return `False` before building/running the step
sequence. Consume/clear the gate after checking so stale validation cannot be
reused by a retry.

Runs with `bed_collision:=false` bypass this gate to preserve simulation.

### Step 4: Verify

Run:

```bash
python3 -m unittest discover \
  -s src/yahboom_rosmaster/gemini_pick_place_executor/test \
  -p 'test_pick_preflight.py' -v
python3 -m py_compile \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/pick_preflight.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/launch/executor.launch.py
git diff --check
```

Inspect the control flow to confirm no `01_home` step is reachable on a failed
or missing gate.

### Step 5: Commit

```bash
git add \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
git commit -m "feat: gate pick motion on post-drive collision check"
```

## Task 6: Continue after an IK-successful orientation fails planning

**Files:**

- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py`
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py`

### Step 1: Confirm the pure test is red against integration behavior

The Task 1 orientation helper test must assert call order:

```python
attempted = []
winner = run_orientation_attempts(
    [0, 1, 2],
    lambda index: attempted.append(index) or index == 1,
)
self.assertEqual(winner, 1)
self.assertEqual(attempted, [0, 1])
```

If this helper already passes, add an AST/source regression test asserting
`_plan_and_execute_pose_once` does not directly `return
self.plan_and_execute(...)` inside the explicit orientation loop.

### Step 2: Change the pose attempt loop

In `_plan_and_execute_pose_once`:

- after IK success, call `plan_and_execute`;
- return `True` only on execution success;
- on planning/execution failure, log
  `orientation N IK passed but planning failed; trying next orientation`;
- continue the loop;
- attempt position-only fallback only after all explicit orientations fail;
- keep OMPL collision checking active for all attempts.

Do not let position-only fallback bypass the protected pick gate.

### Step 3: Verify

Run all helper tests, `py_compile`, and `git diff --check`.

### Step 4: Commit

```bash
git add \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
git commit -m "fix: try remaining grasp orientations after plan failure"
```

## Task 7: Final verification and Jetson handoff

**Files:**

- Modify if needed: `docs/superpowers/specs/2026-07-27-collision-aware-pick-preflight-design.md`

### Step 1: Run the complete local verification

```bash
python3 -m unittest discover \
  -s src/yahboom_rosmaster/gemini_pick_place_executor/test \
  -p 'test_*.py' -v
python3 -m compileall -q \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts \
  src/yahboom_rosmaster/gemini_pick_place_executor/launch
git diff --check
git status --short
```

### Step 2: Review acceptance criteria

Trace the protected pick path and confirm:

- floor-clamped descent reaches the IK search;
- no candidate drive occurs before predicted-bed collision validation;
- collision API failures abort;
- lidar correction changes returned total drive and all tracked points;
- post-drive validation occurs before `_run_pick_phase`;
- missing/failed validation stops before `01_home`;
- plan failure advances to another orientation;
- placement and `bed_collision:=false` behavior remain compatible.

### Step 3: Document Jetson-only verification commands

After the user pulls the branch to the Jetson:

```bash
colcon build --packages-select gemini_pick_place_executor --symlink-install
source install/setup.bash
ros2 launch gemini_pick_place_executor executor.launch.py execute:=false \
  bed_collision:=true collision_preflight_candidates:=5
```

Then perform a guarded hardware run with the printer/arm workspace clear and
the user's normal full launch arguments. The expected log sequence is:

```text
IK-reachable candidate ...
collision-preflight accepted/rejected ...
collision-validated base offset ...
lidar correction ... total ...
post-drive collision revalidation passed
[01_home] ...
```

Any scene API error, missing bed anchor, collision rejection, or post-drive
failure must stop before `[01_home]`.

### Step 4: Final branch review

Run:

```bash
git log --oneline --decorate -8
git diff pragmatic...HEAD --stat
git diff pragmatic...HEAD --check
```

Do not push or merge unless the user requests it.
