"""Double-click launcher; reuse the local server and preserve its running jobs."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
import socket
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener

from .paths import ROOT, UI, DATA

URL = "http://127.0.0.1:8767/"
HTTP = build_opener(ProxyHandler({}))


def ready() -> bool:
    try:
        with HTTP.open(URL + "api/health", timeout=1) as response:
            status = json.load(response)
        if not isinstance(status, dict) or status.get("status") != "ready" or "step_workers" not in status:
            return False
        with HTTP.open(URL, timeout=1) as response:
            return b"DBF STUDIO" in response.read(4096)
    except (OSError, URLError, ValueError):
        return False


def port_in_use() -> bool:
    with socket.socket() as probe:
        probe.settimeout(1)
        return probe.connect_ex(("127.0.0.1", 8767)) == 0


def ensure_server() -> None:
    if ready():
        print("Using the running DBF Studio server.", flush=True)
        return
    process = None
    log_path = None
    if port_in_use():
        print("Port 8767 is in use; checking whether DBF Studio is starting...", flush=True)
    else:
        python = ROOT / ".venv/Scripts/python.exe"
        three = UI / "node_modules/three/build/three.module.js"
        for required in (python, three, UI / "index.html"):
            if not required.is_file():
                raise RuntimeError(f"Required file is missing: {required}\nSee ui/README.md for setup.")
        logs = DATA / "launcher"
        logs.mkdir(parents=True, exist_ok=True)
        log_path = logs / f"server-{datetime.now():%Y%m%d-%H%M%S-%f}.log"
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        with log_path.open("wb") as log:
            process = subprocess.Popen(
                [str(python), "-u", "-m", "dbf_studio.server", "--port", "8767", "--workers", "2"],
                cwd=ROOT,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=log,
                creationflags=subprocess.CREATE_NO_WINDOW,
            )
        print(f"Starting DBF Studio. Log: {log_path}", flush=True)
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        if ready():
            print("DBF Studio is ready.", flush=True)
            return
        if process is not None and process.poll() is not None:
            raise RuntimeError(f"Server exited with code {process.returncode}. Log: {log_path}")
        time.sleep(0.5)
    if log_path:
        raise RuntimeError(f"Server has not become ready yet. Check: {log_path}\nYou can run this launcher again.")
    raise RuntimeError("Port 8767 is occupied but the DBF Studio page is not ready. No process was stopped.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-browser", action="store_true", help="Start/check the server without opening a browser.")
    args = parser.parse_args()
    try:
        ensure_server()
        if not args.no_browser:
            os.startfile(URL)
        print(URL)
        return 0
    except (OSError, RuntimeError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
