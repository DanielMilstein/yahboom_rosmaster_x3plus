# robot_gateway

HTTP gateway that lets the Autoprint platform trigger print-removal jobs on
the Yahboom X3 Plus arm. Each `POST /jobs` spawns
`ros2 launch robot_gateway removal.launch.py` (the Gemini pick-and-place
executor **plus** the Gemini bridge) in its own process group, watches the log
for the executor's completion sentinels, and shuts the launch down with SIGINT
when the sequence ends.

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
already running; the gateway only launches the executor + bridge per job.

## API

| Route | Description |
|---|---|
| `GET /health` | `{ok, busy}` |
| `POST /jobs` | `202 {job_id}`, or `409` if a job is running. Body: `{task, params, gemini_api_keys, execute?, timeout_sec?}` |
| `GET /jobs/{id}` | `{job_id, status, started_at, finished_at, log_tail}` — status: `queued\|running\|succeeded\|failed\|cancelled` |
| `POST /jobs/{id}/cancel` | SIGINTs the launch |

`params` entries become `name:=value` launch arguments (names validated
against `^[a-z][a-z0-9_]*$`; unknown names make the launch fail fast, which
surfaces in `log_tail`). Useful hardware overrides (see
`executor.launch.py` FORWARDED_PARAMS): `drive_axes:=xy`,
`reperceive_after_drive:=false`, `grasp_roll_offset_rad:=1.5708`.

Dry-run without moving the arm: pass `"execute": false` in the body.

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
curl -X POST localhost:8090/jobs -H 'content-type: application/json' \
  -d '{"task":"pick the printed part off the print bed and place it in the bin",
       "params":{}, "gemini_api_keys":["AIza..."], "execute": false}'
curl localhost:8090/jobs/<job_id>
```
