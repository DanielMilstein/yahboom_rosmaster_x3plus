"""Subprocess wrapper for one robot job.

Spawns the job's command in its own process group, tails the output into a
rolling log, and tears the process group down with SIGINT (SIGKILL fallback)
once the job ends. Two kinds, differing in command *and* in how completion is
detected:

    pick_place        `ros2 launch robot_gateway removal.launch.py ...`
                      The executor node keeps spinning after it finishes, so a
                      log sentinel is the only completion signal.
    return_to_origin  `ros2 run gemini_pick_place_executor return_to_origin.py ...`
                      The script terminates on its own, so its exit code is the
                      verdict and there is no sentinel to wait for.
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
import threading
import uuid
from collections import deque
from datetime import datetime, timezone

SENTINEL_SUCCESS = 'Pick-and-place sequence completed'
SENTINEL_FAILURE = 'Pick-and-place sequence aborted'

PARAM_NAME = re.compile(r'^[a-z][a-z0-9_]*$')

KIND_PICK_PLACE = 'pick_place'
KIND_RETURN_TO_ORIGIN = 'return_to_origin'

DEFAULT_TIMEOUT_SEC = 600
SIGKILL_GRACE_SEC = 10
LOG_TAIL_LINES = 200


class RobotJob:
    """One subprocess-per-job lifecycle: queued -> running -> succeeded/failed/cancelled."""

    def __init__(self, task: str = '', params: dict[str, str] | None = None,
                 gemini_api_keys: list[str] | None = None, execute: bool = True,
                 timeout_sec: int = DEFAULT_TIMEOUT_SEC, kind: str = KIND_PICK_PLACE):
        self.id = str(uuid.uuid4())
        self.status = 'queued'
        self.started_at: str | None = None
        self.finished_at: str | None = None
        self.log_tail: deque[str] = deque(maxlen=LOG_TAIL_LINES)
        self._kind = kind
        self._task = task
        self._params = params or {}
        self._keys = gemini_api_keys or []
        self._execute = execute
        self._timeout_sec = timeout_sec
        self._proc: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()

    # -- public API -----------------------------------------------------

    def start(self) -> None:
        cmd = self._build_command()

        env = dict(os.environ)
        if self._kind == KIND_PICK_PLACE:
            # Keys exist only to reach the Gemini bridge inside the launch; the
            # return-to-origin script talks to no API and gets an empty key list.
            for i, key in enumerate(self._keys[:3]):
                env['GEMINI_API_KEY' if i == 0 else f'GEMINI_API_KEY_{i + 1}'] = key

        self._proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,  # own process group so SIGINT reaches every launched node
        )
        self.status = 'running'
        self.started_at = _now()
        threading.Thread(target=self._watch, daemon=True).start()

    def cancel(self) -> None:
        with self._lock:
            if self.status != 'running':
                return
            self._finish('cancelled')
        self._teardown()

    def as_dict(self) -> dict:
        return {
            'job_id': self.id,
            'status': self.status,
            'started_at': self.started_at,
            'finished_at': self.finished_at,
            'log_tail': list(self.log_tail),
        }

    # -- internals ------------------------------------------------------

    def _build_command(self) -> list[str]:
        if self._kind == KIND_RETURN_TO_ORIGIN:
            # Needs yahboom_bridge_node already running and the executor stopped. The
            # single-job lock guarantees the latter, since each job's launch is torn
            # down when the job ends. Never restart the bridge to satisfy this: it
            # integrates odometry from its own start, so restarting re-zeros the origin
            # at the robot's current spot and makes the move a no-op.
            cmd = ['ros2', 'run', 'gemini_pick_place_executor', 'return_to_origin.py']
            # `ros2 run` wants ROS params after --ros-args, one -p per entry — not the
            # bare name:=value form `ros2 launch` takes.
            if self._params:
                cmd.append('--ros-args')
                for name, value in self._validated_params():
                    cmd += ['-p', f'{name}:={value}']
            return cmd

        cmd = ['ros2', 'launch', 'robot_gateway', 'removal.launch.py',
               f'execute:={"true" if self._execute else "false"}',
               f'task:={self._task}']
        for name, value in self._validated_params():
            cmd.append(f'{name}:={value}')
        return cmd

    def _validated_params(self):
        for name, value in self._params.items():
            if not PARAM_NAME.match(name):
                raise ValueError(f'invalid parameter name: {name!r}')
            yield name, value

    def _watch(self) -> None:
        assert self._proc and self._proc.stdout
        # A hung job produces no output, so the timeout must not depend on lines arriving.
        watchdog = threading.Timer(self._timeout_sec, self._on_timeout)
        watchdog.daemon = True
        watchdog.start()
        result: str | None = None

        for line in self._proc.stdout:
            self.log_tail.append(line.rstrip())
            if self._kind == KIND_PICK_PLACE:
                if SENTINEL_SUCCESS in line:
                    result = 'succeeded'
                    break
                if SENTINEL_FAILURE in line:
                    result = 'failed'
                    break

        if self._kind == KIND_RETURN_TO_ORIGIN:
            # This script exits on its own, so the exit code is the verdict — here
            # end-of-stream is normal completion, not the failure it means for a launch.
            # Waiting before cancelling the watchdog keeps a process that closed stdout
            # without exiting covered by the timeout.
            try:
                result = 'succeeded' if self._proc.wait(timeout=SIGKILL_GRACE_SEC) == 0 else 'failed'
            except subprocess.TimeoutExpired:
                result = 'failed'

        watchdog.cancel()
        with self._lock:
            if self.status == 'running':
                # A sentinel matched, the exit code decided, or the stream ended
                # (a launch died on its own).
                self._finish(result or 'failed')
        self._teardown()

    def _on_timeout(self) -> None:
        with self._lock:
            if self.status != 'running':
                return
            self.log_tail.append(f'[gateway] job timed out after {self._timeout_sec}s')
            self._finish('failed')
        self._teardown()

    def _finish(self, status: str) -> None:
        self.status = status
        self.finished_at = _now()

    def _teardown(self) -> None:
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=SIGKILL_GRACE_SEC)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()
