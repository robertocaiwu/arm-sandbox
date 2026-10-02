"""Rerun web viewer serves over HTTP and accepts data over gRPC.

Plan 02's arm_sandbox_viz depends on these ports and this CLI flag
(Rerun 0.38.1 defaults: web viewer 9090, gRPC 9876, `rerun --serve-web`).
"""

import os
import signal
import subprocess
import time
import urllib.request

import pytest

WEB_VIEWER_URL = "http://localhost:9090"
STARTUP_TIMEOUT_S = 30.0
RERUN_VENV_PYTHON = "/opt/rerun/bin/python"
LOG_ONE_POINT = (
    "import rerun as rr; rr.init('arm_sandbox_smoke'); rr.connect_grpc(); "
    "rr.log('smoke/origin', rr.Points3D([[0, 0, 0]])); rr.disconnect()"
)


def wait_for_http(url: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=2) as response:
                if response.status == 200:
                    return True
        except OSError:
            time.sleep(0.5)
    return False


@pytest.fixture
def rerun_server():
    # `rerun` on PATH is a Python wrapper that spawns the real viewer binary: run it in its own
    # process group and stop the whole group, or the viewer survives and keeps the ports.
    process = subprocess.Popen(
        ["rerun", "--serve-web"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        yield process
    finally:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def test_web_viewer_serves_and_accepts_data(rerun_server: subprocess.Popen) -> None:
    assert wait_for_http(WEB_VIEWER_URL, STARTUP_TIMEOUT_S), "web viewer did not come up"
    # rerun-sdk lives in its own venv (it needs NumPy 2; ROS Python stays on NumPy 1.x).
    result = subprocess.run(
        [RERUN_VENV_PYTHON, "-c", LOG_ONE_POINT], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    assert rerun_server.poll() is None, "rerun server exited while receiving data"
