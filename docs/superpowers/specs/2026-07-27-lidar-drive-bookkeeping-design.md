# Lidar Drive Bookkeeping Design

## Problem

After a base drive, the executor compares the requested odometry displacement
with a lidar scan-match measurement and may command a correction for the
measured shortfall. The current bookkeeping adds that correction to the
requested displacement. This double-counts the shortfall because the requested
value was not the physical displacement measured by lidar.

In the failing hardware run, a requested `0.270 m` drive measured `0.248 m`,
then commanded a `+0.016 m` correction. The best final estimate was therefore
`0.264 m`, but the executor recorded `0.286 m`. The resulting `0.022 m` target
frame error caused post-drive IK rejection and made the fallback displacement
use the wrong direction.

## Motion Estimate Contract

Each completed drive records:

- the requested displacement;
- an optional high-confidence lidar-measured displacement for the initial
  drive; and
- the corrective displacement that was subsequently commanded.

The effective displacement is:

```text
(measured displacement if trusted, otherwise requested displacement)
    + corrective displacement
```

The same effective displacement updates scene points and cumulative fallback
positioning.

## Confidence and Failure Handling

- A scan match is trusted only when it passes the existing RMS, correspondence,
  and yaw confidence gates.
- Missing, unreliable, or deliberately skipped lidar measurements fall back to
  the requested odometry displacement.
- A successfully commanded correction is added to the trusted measured
  displacement.
- If corrective driving fails, the effective displacement remains unknown and
  the protected pick aborts fail-closed.
- Collision validation and the arm-motion safety gate remain unchanged.

## Implementation Boundary

The pure `DriveDelta` helper will represent the optional measured displacement
and compute the effective total. The lidar audit will return this completed
drive result rather than returning only a correction tuple. Both staging and
collision-aware final drives will consume the same result, avoiding divergent
coordinate bookkeeping.

No perception, IK, collision geometry, joint-margin, or lidar confidence
thresholds change in this fix.

## Tests

Pure regression tests will verify:

- requested `0.270`, measured `0.248`, correction `0.016` produces `0.264`;
- a measured lateral shortfall plus its correction produces the intended final
  lateral displacement;
- missing or untrusted measurement uses the requested displacement;
- a failed correction leaves the drive result unknown;
- target and destination points use the same effective displacement exactly
  once.

The existing collision-preflight tests and Python compilation checks must
remain green.
