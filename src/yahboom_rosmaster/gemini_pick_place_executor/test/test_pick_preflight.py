import ast
import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from pick_preflight import (  # noqa: E402
    DriveDelta,
    IKCandidate,
    all_states_collision_free,
    apply_drive_delta_xy,
    arm_motion_allowed,
    choose_collision_validated_candidates,
    complete_drive_delta,
    ordered_offset_shortlist,
    ordered_shortlist,
    run_orientation_attempts,
    untried_candidates,
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

    def test_no_lidar_correction_keeps_requested_drive(self):
        delta = complete_drive_delta(0.24, -0.01, (0.0, 0.0))

        self.assertIsNotNone(delta)
        self.assertEqual(delta.total, (0.24, -0.01))

    def test_failed_lidar_correction_makes_drive_bookkeeping_unknown(self):
        delta = complete_drive_delta(0.24, 0.0, None)

        self.assertIsNone(delta)

    def test_same_total_drive_updates_target_and_destination_once(self):
        delta = complete_drive_delta(0.24, 0.0, (0.013, -0.004))

        target = apply_drive_delta_xy(0.70, -0.02, delta)
        destination = apply_drive_delta_xy(0.91, 0.12, delta)

        self.assertAlmostEqual(target[0], 0.447)
        self.assertAlmostEqual(target[1], -0.016)
        self.assertAlmostEqual(destination[0], 0.657)
        self.assertAlmostEqual(destination[1], 0.124)

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

    def test_shortlist_preserves_offset_orientation_order_and_solutions(self):
        candidates = [
            IKCandidate(0.24, 0.0, 0, ("a0", "a1"), (0.255, 0.195)),
            IKCandidate(0.24, 0.0, 2, ("b0", "b1"), (0.255, 0.195)),
            IKCandidate(0.27, 0.0, 1, ("c0", "c1"), (0.255, 0.195)),
        ]

        shortlisted = ordered_shortlist(candidates, 2)

        self.assertEqual(shortlisted, candidates[:2])
        self.assertEqual(shortlisted[1].orientation_index, 2)
        self.assertEqual(shortlisted[1].joint_solutions, ("b0", "b1"))
        self.assertEqual(shortlisted[1].fingertip_zs, (0.255, 0.195))

    def test_shortlist_limit_is_clamped_to_one(self):
        candidates = [
            IKCandidate(0.24, 0.0, 0, ("a0", "a1"), (0.255, 0.195)),
            IKCandidate(0.27, 0.0, 1, ("b0", "b1"), (0.255, 0.195)),
        ]

        self.assertEqual(ordered_shortlist(candidates, 0), candidates[:1])

    def test_offset_shortlist_keeps_orientations_and_reaches_later_offset(self):
        candidates = [
            IKCandidate(0.24, 0.0, 0, ("a0", "a1"), (0.255, 0.195)),
            IKCandidate(0.24, 0.0, 2, ("b0", "b1"), (0.255, 0.195)),
            IKCandidate(0.27, 0.0, 1, ("c0", "c1"), (0.255, 0.195)),
            IKCandidate(0.30, 0.0, 0, ("d0", "d1"), (0.255, 0.195)),
        ]

        shortlisted = ordered_offset_shortlist(candidates, 2)

        self.assertEqual(shortlisted, candidates[:3])

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

    def test_pick_collision_rejects_candidate_even_when_pre_pick_is_clear(self):
        states = ["pre-pick", "pick"]

        safe = all_states_collision_free(
            states,
            lambda state: state == "pick",
        )

        self.assertFalse(safe)

    def test_both_pick_states_clear_accepts_candidate(self):
        self.assertTrue(
            all_states_collision_free(
                ["pre-pick", "pick"],
                lambda _state: False,
            )
        )

    def test_collision_query_exception_fails_states_closed(self):
        def unavailable(_state):
            raise RuntimeError("planning scene query failed")

        self.assertFalse(
            all_states_collision_free(["pre-pick", "pick"], unavailable)
        )

    def test_arm_gate_opens_only_for_literal_post_drive_success(self):
        self.assertTrue(arm_motion_allowed(True))
        self.assertFalse(arm_motion_allowed(False))
        self.assertFalse(arm_motion_allowed(None))
        self.assertFalse(arm_motion_allowed(1))

    def test_candidate_fallback_does_not_repeat_attempted_candidate(self):
        candidates = [
            IKCandidate(0.24, 0.0, 0, ("a0", "a1"), (0.255, 0.195)),
            IKCandidate(0.24, 0.0, 2, ("b0", "b1"), (0.255, 0.195)),
            IKCandidate(0.27, 0.0, 1, ("c0", "c1"), (0.255, 0.195)),
        ]

        remaining = untried_candidates(candidates, {candidates[0]})

        self.assertEqual(remaining, candidates[1:])

    def test_planning_failure_after_ik_success_tries_next_orientation(self):
        attempted = []

        winner = run_orientation_attempts(
            [0, 1, 2],
            lambda index: attempted.append(index) or index == 1,
        )

        self.assertEqual(winner, 1)
        self.assertEqual(attempted, [0, 1])


class ExecutorSourceRegressionTests(unittest.TestCase):
    def _method(self, name):
        executor_path = (
            SCRIPTS_DIR / "gemini_pick_place_executor.py"
        )
        tree = ast.parse(executor_path.read_text(encoding="utf-8"))
        return next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
            and node.name == name
        )

    def test_pose_loop_does_not_return_the_first_plan_attempt_directly(self):
        method = self._method("_plan_and_execute_pose_once")
        direct_plan_returns = [
            node
            for node in ast.walk(method)
            if isinstance(node, ast.Return)
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Attribute)
            and node.value.func.attr == "plan_and_execute"
        ]

        self.assertEqual(direct_plan_returns, [])

    def test_collision_preflight_models_the_open_gripper(self):
        method = self._method("_candidate_is_collision_free")
        open_gripper_assignments = [
            node
            for node in ast.walk(method)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "set_variable_position"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "grip_joint"
        ]

        self.assertNotEqual(open_gripper_assignments, [])

    def test_real_bed_is_applied_synchronously_for_final_planning(self):
        method = self._method("_publish_printer_bed_collision")
        called_attributes = {
            node.func.attr
            for node in ast.walk(method)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
        }

        self.assertIn("read_write", called_attributes)
        self.assertIn("apply_collision_object", called_attributes)

    def test_collision_box_removal_updates_the_local_scene_synchronously(self):
        method = self._method("_remove_collision_box")
        called_attributes = {
            node.func.attr
            for node in ast.walk(method)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
        }

        self.assertIn("read_write", called_attributes)
        self.assertIn("apply_collision_object", called_attributes)


if __name__ == "__main__":
    unittest.main()
