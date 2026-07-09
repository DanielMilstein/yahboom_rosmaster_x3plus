"""Subprocess wrapper for one removal job.

Spawns `ros2 launch robot_gateway removal.launch.py ...` in its own process
group, tails the output for the executor's completion sentinels, and tears the
launch down with SIGINT (SIGKILL fallback) once the sequence ends — the
executor node keeps spinning after it finishes, so the log sentinel is the
only completion signal.
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from datetime import datetime, timezone

SENTINEL_SUCCESS = 'Pick-and-place sequence completed'
SENTINEL_FAILURE = 'Pick-and-place sequence aborted'

LAUNCH_ARG_NAME = re.compile(r'^[a-z][a-z0-9_]*$')

DEFAULT_TIMEOUT_SEC = 600
SIGKILL_GRACE_SEC = 10
LOG_TAIL_LINES = 200


class RemovalJob:
    """One launch-per-job lifecycle: queued -> running -> succeeded/failed/cancelled."""

    def __init__(self, task: str, params: dict[str, str], gemini_api_keys: list[str],
                 execute: bool = True, timeout_sec: int = DEFAULT_TIMEOUT_SEC):
        self.id = str(uuid.uuid4())
        self.status = 'queued'
        self.started_at: str | None = None
        self.finished_at: str | None = None
        self.log_tail: deque[str] = deque(maxlen=LOG_TAIL_LINES)
        self._task = task
        self._params = params
        self._keys = gemini_api_keys
        self._execute = execute
        self._timeout_sec = timeout_sec
        self._proc: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()

    # -- public API -----------------------------------------------------

    def start(self) -> None:
        cmd = ['ros2', 'launch', 'robot_gateway', 'removal.launch.py',
               f'execute:={"true" if self._execute else "false"}',
               f'task:={self._task}']
        for name, value in self._params.items():
            if not LAUNCH_ARG_NAME.match(name):
                raise ValueError(f'invalid launch parameter name: {name!r}')
            cmd.append(f'{name}:={value}')

        env = dict(os.environ)
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

    def _watch(self) -> None:
        assert self._proc and self._proc.stdout
        # A hung launch produces no output, so the timeout must not depend on lines arriving.
        watchdog = threading.Timer(self._timeout_sec, self._on_timeout)
        watchdog.daemon = True
        watchdog.start()
        result: str | None = None

        for line in self._proc.stdout:
            self.log_tail.append(line.rstrip())
            if SENTINEL_SUCCESS in line:
                result = 'succeeded'
                break
            if SENTINEL_FAILURE in line:
                result = 'failed'
                break

        watchdog.cancel()
        with self._lock:
            if self.status == 'running':
                # Either a sentinel matched, or the stream ended (launch died on its own).
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
