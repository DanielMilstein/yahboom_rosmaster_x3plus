# Lidar Drive Bookkeeping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make post-drive target coordinates and fallback positioning use trusted lidar-measured motion plus corrective motion, without double-counting the requested displacement.

**Architecture:** Extend the ROS-independent `DriveDelta` value to distinguish requested, measured, and corrective translations. Make `_lidar_audit_drive()` return that completed value so staging and collision-aware final drives consume one shared motion estimate, while unreliable lidar data continues to fall back to odometry.

**Tech Stack:** Python 3, `dataclasses`, ROS 2 Humble, MoveItPy, `unittest`

---

### Task 1: Represent measured base motion

**Files:**
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/pick_preflight.py:10-62`
- Test: `src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py:24-58`

- [x] **Step 1: Write the failing measured-motion regression tests**

Replace the old correction-total expectation and add odometry fallback coverage:

```python
def test_lidar_measurement_and_correction_define_total_drive(self):
    delta = complete_drive_delta(
        0.270,
        0.0,
        (0.016, 0.0),
        measured=(0.248, 0.002),
    )

    self.assertAlmostEqual(delta.total[0], 0.264)
    self.assertAlmostEqual(delta.total[1], 0.002)

def test_missing_lidar_measurement_falls_back_to_requested_drive(self):
    delta = complete_drive_delta(0.270, -0.01, (0.0, 0.0))

    self.assertEqual(delta.total, (0.270, -0.01))

def test_same_measured_total_updates_scene_points_once(self):
    delta = complete_drive_delta(
        0.270,
        0.0,
        (0.016, -0.004),
        measured=(0.248, 0.002),
    )

    target = apply_drive_delta_xy(0.633, 0.011, delta)
    destination = apply_drive_delta_xy(0.716, -0.313, delta)

    self.assertAlmostEqual(target[0], 0.369)
    self.assertAlmostEqual(target[1], 0.013)
    self.assertAlmostEqual(destination[0], 0.452)
    self.assertAlmostEqual(destination[1], -0.311)
```

- [x] **Step 2: Run the regression test and verify RED**

Run:

```bash
python3 -m unittest src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
```

Expected: FAIL because `complete_drive_delta()` does not accept `measured` and the current total incorrectly uses requested plus correction.

- [x] **Step 3: Implement measured-motion semantics**

Extend the helper without changing its fail-closed `None` result:

```python
@dataclass(frozen=True)
class DriveDelta:
    """Best estimate of one completed base translation."""

    requested_dx: float
    requested_dy: float
    correction_dx: float = 0.0
    correction_dy: float = 0.0
    measured_dx: float = None
    measured_dy: float = None

    @property
    def correction(self):
        return float(self.correction_dx), float(self.correction_dy)

    @property
    def total(self):
        base_dx = (
            self.requested_dx
            if self.measured_dx is None
            else self.measured_dx
        )
        base_dy = (
            self.requested_dy
            if self.measured_dy is None
            else self.measured_dy
        )
        return (
            float(base_dx) + float(self.correction_dx),
            float(base_dy) + float(self.correction_dy),
        )


def complete_drive_delta(
    requested_dx,
    requested_dy,
    correction,
    measured=None,
):
    """Build a known drive estimate, or ``None`` after failed correction."""

    if correction is None:
        return None
    correction_dx, correction_dy = correction
    measured_dx, measured_dy = (
        (None, None) if measured is None else measured
    )
    return DriveDelta(
        requested_dx,
        requested_dy,
        correction_dx,
        correction_dy,
        measured_dx,
        measured_dy,
    )
```

- [x] **Step 4: Run the helper suite and verify GREEN**

Run:

```bash
python3 -m unittest src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
```

Expected: all tests pass.

### Task 2: Propagate the completed lidar estimate

**Files:**
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py:2627-2769`
- Modify: `src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py:3004-3037`
- Test: `src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py`

- [x] **Step 1: Add source-regression coverage for the shared result**

Add an AST test requiring both drive consumers to use the result returned by
`_lidar_audit_drive()` directly rather than wrapping its correction a second
time:

```python
def test_lidar_audit_returns_completed_drive_delta(self):
    method = self._method("_lidar_audit_drive")
    helper_calls = [
        node
        for node in ast.walk(method)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "complete_drive_delta"
    ]

    self.assertNotEqual(helper_calls, [])
```

- [x] **Step 2: Run the regression test and verify RED**

Run:

```bash
python3 -m unittest src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
```

Expected: FAIL because `_lidar_audit_drive()` still returns only correction
tuples.

- [x] **Step 3: Return one completed motion estimate**

Change `_lidar_audit_drive()` so:

```python
odometry_delta = lambda: complete_drive_delta(
    applied_dx, applied_dy, (0.0, 0.0)
)
```

is returned when scans are missing or untrusted. After a match, define:

```python
translation_confident = (
    rms <= 0.035 and n_pairs >= 100 and abs(dyaw) <= 0.05
)
measured = (mdx, mdy) if translation_confident else None
```

Return `complete_drive_delta(applied_dx, applied_dy, (0.0, 0.0), measured)`
when no correction is needed or correction is disabled. After a successful
correction, return:

```python
return complete_drive_delta(
    applied_dx,
    applied_dy,
    (cx, cy),
    measured=(mdx, mdy),
)
```

Keep returning `None` after a failed correction drive.

- [x] **Step 4: Consume the shared result in both drive paths**

In `drive_staging()`, assign:

```python
delta = self._lidar_audit_drive(lidar_pts_before, dx, dy, label)
```

In `drive_to_feasible()`, use the same assignment after motion, and use
`complete_drive_delta(requested_dx, requested_dy, (0.0, 0.0))` only when no
motion was requested. In both methods, log `delta.correction` and use
`delta.total` for scene-point and cumulative-offset updates.

- [x] **Step 5: Run all verification commands**

Run:

```bash
python3 -m unittest src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
python3 -m compileall -q src/yahboom_rosmaster/gemini_pick_place_executor/scripts src/yahboom_rosmaster/gemini_pick_place_executor/launch
git diff --check
```

Expected: all tests pass, compilation exits zero, and `git diff --check`
produces no output.

- [x] **Step 6: Commit and push**

```bash
git add docs/superpowers/plans/2026-07-27-lidar-drive-bookkeeping.md \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/pick_preflight.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/scripts/gemini_pick_place_executor.py \
  src/yahboom_rosmaster/gemini_pick_place_executor/test/test_pick_preflight.py
git commit -m "fix: use measured lidar motion for drive bookkeeping"
git push origin HEAD:refs/heads/collision-aware-pick-preflight
```
