# Collision-Aware Pick Preflight Design

## Context

The executor currently selects a base displacement by searching for a target
pose that has an inverse-kinematics solution at the pre-pick and pick heights.
The search deliberately calls `_ik_solve(..., check_collision=False)` because
performing planning-scene queries for every state in the full search grid
previously caused MoveItPy planning-scene-monitor concurrency crashes.

This makes the search's “feasible” result kinematic only. The printer-bed
collision slab is published later, after the base drive, when the pick attempt
starts. A selected pose can therefore be reachable but collide with the bed.
The July 27 hardware run demonstrated this twice: the base search accepted an
offset, the pre-pick succeeded, and OMPL rejected the final pick goal because
`arm_link5` contacted `printer_bed`.

The search also validates the nominal grasp descent rather than the final
floor-clamped descent, and lidar correction drives are not reflected in the
tracked target coordinates or revalidated before arm motion.

## Goals

- Do not drive to a pick position unless the final, floor-clamped grasp has a
  collision-free arm configuration with respect to the printer bed.
- Preserve the fast IK-only grid search and avoid returning to hundreds of
  planning-scene queries.
- Validate one consistent orientation at both pre-pick and pick poses.
- Account for lidar correction displacement before the arm moves.
- If one orientation passes IK but fails MoveIt planning, continue trying
  other orientations before failing the pick attempt.
- Produce explicit logs that distinguish kinematic reachability, collision
  validation, planning validation, and post-drive revalidation.

## Non-goals

- General obstacle avoidance or live octomap integration.
- Navigation or localization changes.
- Supporting variable printer, wall, container, or object geometry.
- Replacing the existing printer-bed slab model.
- Changing the grasp or gripper-contact algorithms beyond preventing bed
  contact from reaching them.

## Selected Approach: Two-Stage Candidate Selection

### Stage 1: Kinematic shortlist

Retain the current grid search as a kinematic filter. Instead of treating its
first IK result as fully feasible, represent it as a candidate containing:

- base displacement `(dx, dy)`;
- orientation index;
- pre-pick and pick joint solutions;
- target poses in the hypothetical post-drive `base_footprint` frame;
- the score already used by `base_search_order`.

The search must use the final grasp descent returned by
`_clamp_grasp_descent`, not `_grasp_descent_nominal`. This makes the searched
pick pose identical to the one the executor will request.

The search will gather a small ordered shortlist rather than querying the
planning scene throughout the entire grid. The shortlist size will be a
parameter with a conservative default of five candidates.

### Stage 2: Serialized collision preflight

Validate shortlisted candidates one at a time:

1. Place a temporary printer-bed slab at the position it would occupy after
   the candidate base displacement.
2. Wait for the MoveIt planning scene to observe that exact slab revision.
3. Collision-check the candidate's pre-pick and pick joint states against the
   same planning scene, using the same arm group used during execution.
4. Accept the first candidate for which both states are collision-free.
5. Remove the temporary slab before commanding the base.

Only the shortlist uses planning-scene collision queries. They run
serially—never concurrently—and outside the full grid loop, limiting both
latency and exposure to the earlier MoveItPy concurrency failure.

If the scene update cannot be confirmed or the collision API raises an error,
the candidate fails closed. The robot must not interpret an unavailable
collision checker as safe.

### Post-drive revalidation

The lidar audit will return any corrective translation it commands. Base-drive
bookkeeping will apply both the requested displacement and this correction to
the target and destination coordinates.

After the base finishes all odometry and lidar corrections:

1. Publish the actual printer-bed slab using the final wall estimate and bed
   probe.
2. Recompute the final pre-pick and pick poses in the resulting base frame.
3. Collision-check both states again.
4. If validation fails, keep the arm stowed and select/reposition to another
   shortlisted candidate. Convert that candidate's original displacement into
   a delta from the base's measured current displacement before driving. Do
   not begin `01_home` or `03_pre_pick`.

This guard covers correction error, scan-match adjustments, and scene changes
between hypothetical preflight and physical arrival.

## Pick-Time Orientation Fallback

`_plan_and_execute_pose_once` currently returns immediately after the first
IK-successful orientation calls the planner. If OMPL rejects that goal, the
remaining orientations are never tried.

Change the loop so that:

- IK failure continues to the next orientation, as today.
- IK success followed by planning failure also continues to the next
  orientation.
- The method returns success on the first executed plan.
- Position-only fallback is attempted only after every explicit orientation
  has failed.

For the bed-protected `04_pick` step, position-only fallback must not bypass
orientation collision safety. It may run only through the same planning scene
and must remain collision-checked by OMPL.

## Failure Handling

- No collision-validated candidate: do not drive; abort with a message
  reporting how many candidates failed IK, collision preflight, or scene
  synchronization.
- Temporary scene update unavailable: fail closed and remove the temporary
  object.
- Post-drive validation failure: do not move the arm; attempt a bounded base
  adjustment to the next validated candidate.
- All candidate adjustments exhausted: enter the existing failure-reset path.
- Collision detected during final OMPL planning despite preflight: try the
  remaining orientations and log the preflight/planner disagreement.

## Logging

Replace ambiguous “feasible” wording with:

- `IK-reachable candidate`;
- `collision-preflight rejected`;
- `collision-validated base offset`;
- `post-drive collision revalidation passed/failed`;
- `orientation N IK passed but planning failed; trying next orientation`.

Each accepted candidate log must include `(dx, dy)`, orientation index,
pre-pick/pick fingertip heights, slab top height, and whether lidar correction
changed the final base displacement.

## Testing

Because ROS and MoveItPy are unavailable in the Mac development environment,
extract the candidate bookkeeping and decision logic into pure functions that
can be unit-tested without ROS. Use test doubles for IK, scene synchronization,
and collision results.

Regression tests must cover:

1. The search receives the floor-clamped grasp descent.
2. An IK-reachable candidate with a colliding pick state is rejected.
3. A later orientation at the same base offset can be selected.
4. A later base offset is selected when all orientations at the first collide.
5. Scene-query exceptions fail closed.
6. Lidar correction displacement updates target and destination coordinates.
7. Post-drive collision failure prevents arm motion.
8. An OMPL failure after IK success tries the next orientation.

Static Python syntax checks remain the local baseline. Final validation also
requires a Jetson dry preflight and then a guarded hardware run with the arm
workspace clear.

## Acceptance Criteria

- Logs never call an IK-only result collision-free or fully feasible.
- The robot does not start a candidate drive without preflight against the
  predicted printer-bed slab.
- The robot does not start the pick arm sequence without post-drive
  revalidation against the actual slab.
- The July 27 failure pattern causes candidate rejection before `01_home`,
  rather than an `arm_link5`/`printer_bed` failure at `04_pick`.
- Lidar corrections are reflected in all tracked scene points.
- Failure of the collision-checking mechanism stops the mission safely.
