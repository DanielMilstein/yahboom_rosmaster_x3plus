"""Pure decision helpers for collision-aware pick preflight.

This module deliberately has no ROS or MoveIt imports so its safety-critical
bookkeeping can be exercised on development machines without a ROS install.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class DriveDelta:
    """Requested base translation plus any lidar-commanded correction."""

    requested_dx: float
    requested_dy: float
    correction_dx: float = 0.0
    correction_dy: float = 0.0

    @property
    def total(self):
        return (
            float(self.requested_dx) + float(self.correction_dx),
            float(self.requested_dy) + float(self.correction_dy),
        )


@dataclass(frozen=True)
class IKCandidate:
    """One orientation that solves every required pose at a base offset."""

    dx: float
    dy: float
    orientation_index: int
    joint_solutions: tuple
    fingertip_zs: tuple


def apply_drive_delta_xy(x, y, delta):
    """Express an existing base-frame point after ``delta`` is driven."""

    total_dx, total_dy = delta.total
    return float(x) - total_dx, float(y) - total_dy


def complete_drive_delta(requested_dx, requested_dy, correction):
    """Build a known total drive, or ``None`` if correction motion is unknown."""

    if correction is None:
        return None
    correction_dx, correction_dy = correction
    return DriveDelta(
        requested_dx,
        requested_dy,
        correction_dx,
        correction_dy,
    )


def choose_collision_validated_candidates(candidates, validator):
    """Return safe candidates in order, rejecting validator errors closed."""

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
    """Return the first item whose attempt succeeds, preserving order."""

    for item in items:
        if attempt(item):
            return item
    return None
