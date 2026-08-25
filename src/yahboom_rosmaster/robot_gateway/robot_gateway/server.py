"""HTTP gateway for robot jobs.

Run inside a sourced workspace (systemd unit or terminal):

    ros2 run robot_gateway gateway
    # or: python3 -m robot_gateway.server

Environment:
    ROBOT_GATEWAY_PORT   listen port (default 8090)
    ROBOT_GATEWAY_HOST   bind address (default 0.0.0.0)

API (consumed by the Autoprint platform):
    GET  /health              -> {ok, busy}
    POST /jobs                -> 202 {job_id} | 409 when a job is running
         body: {kind?: 'pick_place'|'return_to_origin', task?: str,
                params: {name: value}, gemini_api_keys: [str],
                execute?: bool, timeout_sec?: int}
         kind defaults to 'pick_place'; task is required for that kind only.
    GET  /jobs/{id}           -> {job_id, status, started_at, finished_at, log_tail}
    POST /jobs/{id}/cancel    -> {ok}
"""

from __future__ import annotations

import os
import threading
from typing import Literal

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from .job import RobotJob

app = FastAPI(title='robot_gateway')

_jobs: dict[str, RobotJob] = {}
_lock = threading.Lock()


class JobRequest(BaseModel):
    # Default is load-bearing: the Autoprint batch path sends no kind.
    kind: Literal['pick_place', 'return_to_origin'] = 'pick_place'
    task: str = ''
    params: dict[str, str] = Field(default_factory=dict)
    gemini_api_keys: list[str] = Field(default_factory=list)
    execute: bool = True
    timeout_sec: int = Field(default=600, ge=30, le=3600)


def _running_job() -> RobotJob | None:
    return next((j for j in _jobs.values() if j.status in ('queued', 'running')), None)


@app.get('/health')
def health() -> dict:
    return {'ok': True, 'busy': _running_job() is not None}


@app.post('/jobs', status_code=202)
def create_job(req: JobRequest) -> dict:
    # Checked here rather than on the model so an empty task is a 400, not pydantic's 422.
    if req.kind == 'pick_place' and not req.task.strip():
        raise HTTPException(status_code=400, detail='task is required for pick_place jobs')
    with _lock:
        if _running_job() is not None:
            raise HTTPException(status_code=409, detail='a robot job is already running')
        job = RobotJob(
            kind=req.kind,
            task=req.task,
            params=req.params,
            gemini_api_keys=req.gemini_api_keys,
            execute=req.execute,
            timeout_sec=req.timeout_sec,
        )
        try:
            job.start()
        except ValueError as err:
            raise HTTPException(status_code=400, detail=str(err))
        _jobs[job.id] = job
    return {'job_id': job.id}


@app.get('/jobs/{job_id}')
def get_job(job_id: str) -> dict:
    job = _jobs.get(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail='unknown job')
    return job.as_dict()


@app.post('/jobs/{job_id}/cancel')
def cancel_job(job_id: str) -> dict:
    job = _jobs.get(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail='unknown job')
    job.cancel()
    return {'ok': True}


def main() -> None:
    import uvicorn

    uvicorn.run(
        app,
        host=os.environ.get('ROBOT_GATEWAY_HOST', '0.0.0.0'),
        port=int(os.environ.get('ROBOT_GATEWAY_PORT', '8090')),
    )


if __name__ == '__main__':
    main()
