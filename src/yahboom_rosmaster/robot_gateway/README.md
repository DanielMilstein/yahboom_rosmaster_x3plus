# robot_gateway

HTTP gateway that lets the Autoprint platform trigger jobs on the Yahboom X3
Plus. Each `POST /jobs` spawns one subprocess in its own process group, tails
its output, and shuts it down with SIGINT when the job ends. Two job kinds are
supported — print removal (the Gemini pick-and-place executor plus the Gemini
bridge) and returning the base to its odometry origin.

## Why subprocess-per-job

The executor is a launch-time state machine: its task and ~77 tunables are
node parameters resolved at launch, and after logging
`Pick-and-place sequence completed` / `aborted` it just keeps spinning.
Launching per job gives a clean lifecycle, and because the Gemini bridge runs
inside the same subprocess, the `GEMINI_API_KEY`, `GEMINI_API_KEY_2`,
`GEMINI_API_KEY_3` environment variables injected from the request body reach
it with no extra plumbing. The gateway never persists keys.

## Build & run

```bash
cd ~/yahboom_rosmaster_x3plus
colcon build --packages-select robot_gateway
source install/setup.bash
pip3 install fastapi uvicorn        # once
ros2 run robot_gateway gateway      # listens on 0.0.0.0:8090
```

Prerequisite: the robot's standing bringup (drivers, controllers, camera) is
already running; the gateway only launches per-job processes.

## API

| Route | Description |
|---|---|
| `GET /health` | `{ok, busy}` |
| `POST /jobs` | `202 {job_id}`, or `409` if a job is running. Body: `{kind?, task?, params, gemini_api_keys, execute?, timeout_sec?}` |
| `GET /jobs/{id}` | `{job_id, status, started_at, finished_at, log_tail}` — status: `queued\|running\|succeeded\|failed\|cancelled` |
| `POST /jobs/{id}/cancel` | SIGINTs the job's process group |

Only one job runs at a time, whatever its kind; the second request gets `409`.

### Job kinds

`kind` defaults to `pick_place`, so callers that omit it are unaffected.

| `kind` | Command | `params` syntax | Completion |
|---|---|---|---|
| `pick_place` (default) | `ros2 launch robot_gateway removal.launch.py` | `name:=value` launch arguments | log sentinel — the executor keeps spinning after it finishes |
| `return_to_origin` | `ros2 run gemini_pick_place_executor return_to_origin.py` | `--ros-args -p name:=value` | process exit code — `0` → `succeeded`, non-zero → `failed` |

`task`, `execute` and `gemini_api_keys` apply to `pick_place` only. `task` is
required for it (empty → `400`); `return_to_origin` takes no task and needs no
API keys. Param *names* are validated against `^[a-z][a-z0-9_]*$` for both
kinds; unknown names make the job fail fast, which surfaces in `log_tail`.

Useful `pick_place` hardware overrides (see `executor.launch.py`
FORWARDED_PARAMS): `drive_axes:=xy`, `reperceive_after_drive:=false`,
`grasp_roll_offset_rad:=1.5708`.

Dry-run without moving the arm: pass `"execute": false` in the body.

#### `return_to_origin` prerequisites

The script drives the base closed-loop on `/odom` back to the origin, so it
needs `yahboom_bridge_node` **already running** and the executor stopped. The
single-job lock guarantees the executor is down, because each job's launch is
torn down when the job ends. Never restart the bridge to satisfy this: it
integrates odometry from its own start, so restarting re-zeros the origin at
the robot's current spot and makes the move a no-op.

Its params (`goal_x`, `goal_y`, `goal_yaw`, `drive_kp`,
`drive_max_lin_speed_mps`, `drive_position_tol_m`, `drive_timeout_sec`, …) are
all declared as **doubles**, so pass `"0.0"`, not `"0"` — an integer literal
fails on a type mismatch.

## systemd unit (on the robot)

```ini
# /etc/systemd/system/robot-gateway.service
[Unit]
Description=Autoprint robot gateway
After=network-online.target

[Service]
User=yahboom
ExecStart=/bin/bash -lc 'source ~/yahboom_rosmaster_x3plus/install/setup.bash && ros2 run robot_gateway gateway'
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

## Smoke test

```bash
curl localhost:8090/health

# print removal (kind omitted — the default)
curl -X POST localhost:8090/jobs -H 'content-type: application/json' \
  -d '{"task":"pick the printed part off the print bed and place it in the bin",
       "params":{}, "gemini_api_keys":["AIza..."], "execute": false}'

# drive the base back to the odometry origin
curl -X POST localhost:8090/jobs -H 'content-type: application/json' \
  -d '{"kind":"return_to_origin", "params":{}, "gemini_api_keys":[]}'

# ...with an explicit goal / slower approach
curl -X POST localhost:8090/jobs -H 'content-type: application/json' \
  -d '{"kind":"return_to_origin",
       "params":{"goal_x":"0.0","goal_y":"0.0","goal_yaw":"0.0",
                 "drive_max_lin_speed_mps":"0.05"}}'

curl localhost:8090/jobs/<job_id>
curl -X POST localhost:8090/jobs/<job_id>/cancel
```
