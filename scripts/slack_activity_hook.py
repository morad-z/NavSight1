#!/usr/bin/env python3
"""Hook wrapper: reads prompt from stdin JSON, calls slack_agent.py auto activity."""
import json
import subprocess
import sys
from pathlib import Path

try:
    data = json.load(sys.stdin)
    prompt = data.get("prompt", "")
except Exception:
    prompt = ""

if prompt:
    agent = Path(__file__).parent.parent / "slack_agent.py"
    subprocess.run(
        [sys.executable, str(agent), "auto", "activity", "--prompt", prompt],
        timeout=10,
        capture_output=True,
    )
