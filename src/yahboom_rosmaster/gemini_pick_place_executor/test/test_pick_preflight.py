import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from pick_preflight import (  # noqa: E402
    DriveDelta,
    IKCandidate,
    apply_drive_delta_xy,
    choose_collision_validated_candidates,
    run_orientation_attempts,
)


class PickPreflightTests(unittest.TestCase):
    def test_lidar_translation_is_part_of_total_drive(self):
        delta = DriveDelta(0.24, 0.01, 0.013, -0.004)

        total_dx, total_dy = delta.total
        point_x, point_y = apply_drive_delta_xy(0.70, -0.02, delta)

        self.assertAlmostEqual(total_dx, 0.253)
        self.assertAlmostEqual(total_dy, 0.006)
        self.assertAlmostEqual(point_x, 0.447)
        self.assertAlmostEqual(point_y, -0.026)

    def test_colliding_candidate_is_rejected_for_later_orientation(self):
        first = IKCandidate(
            0.24, 0.0, 0, ("pre-a", "pick-a"), (0.255, 0.195)
        )
        second = IKCandidate(
            0.24, 0.0, 2, ("pre-b", "pick-b"), (0.255, 0.195)
        )

        accepted, rejected = choose_collision_validated_candidates(
            [first, second],
            lambda candidate: candidate.orientation_index == 2,
        )

        self.assertEqual(accepted, [second])
        self.assertEqual(rejected, 1)

    def test_later_base_offset_is_kept_when_first_offset_collides(self):
        candidates = [
            IKCandidate(0.24, 0.0, 0, ("a0", "a1"), (0.255, 0.195)),
            IKCandidate(0.24, 0.0, 2, ("b0", "b1"), (0.255, 0.195)),
            IKCandidate(0.27, 0.0, 1, ("c0", "c1"), (0.255, 0.195)),
        ]

        accepted, rejected = choose_collision_validated_candidates(
            candidates,
            lambda candidate: candidate.dx == 0.27,
        )

        self.assertEqual(accepted, [candidates[2]])
        self.assertEqual(rejected, 2)

    def test_scene_exception_fails_candidate_closed(self):
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
        attempted = []

        winner = run_orientation_attempts(
            [0, 1, 2],
            lambda index: attempted.append(index) or index == 1,
        )

        self.assertEqual(winner, 1)
        self.assertEqual(attempted, [0, 1])


if __name__ == "__main__":
    unittest.main()
