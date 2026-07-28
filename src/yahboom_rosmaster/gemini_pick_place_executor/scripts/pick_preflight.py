"""Pure decision helpers for collision-aware pick preflight.

This module deliberately has no ROS or MoveIt imports so its safety-critical
bookkeeping can be exercised on development machines without a ROS install.
"""

from dataclasses import dataclass


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


def ordered_shortlist(candidates, limit):
    """Return at least one candidate while preserving the search order."""

    return list(candidates[: max(1, int(limit))])


def ordered_offset_shortlist(candidates, offset_limit):
    """Keep every orientation for the first N distinct base offsets."""

    limit = max(1, int(offset_limit))
    selected = []
    seen_offsets = []
    for candidate in candidates:
        offset = (candidate.dx, candidate.dy)
        if offset not in seen_offsets:
            if len(seen_offsets) >= limit:
                break
            seen_offsets.append(offset)
        selected.append(candidate)
    return selected


def all_states_collision_free(states, is_colliding):
    """Return true only when every state query succeeds and is collision-free."""

    try:
        return all(not is_colliding(state) for state in states)
    except Exception:
        return False


def arm_motion_allowed(post_drive_valid):
    """Open the arm-motion gate only for an explicit successful validation."""

    return post_drive_valid is True


def untried_candidates(candidates, attempted):
    """Return ordered candidates not present in the attempted set."""

    return [
        candidate
        for candidate in candidates
        if candidate not in attempted
    ]


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
