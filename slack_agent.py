#!/usr/bin/env python3
"""
Agent Slack Bridge — Read and post to your team's Slack channel.

Setup:
  bash setup_agent.sh        # Creates .env with your name and tokens

Commands:
  python slack_agent.py read                          # Read last messages
  python slack_agent.py post --message "..."          # Post mid-session update
  python slack_agent.py post --message "..." --final  # Post end-of-session summary
  python slack_agent.py status                        # Post working-on heartbeat
"""

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
import urllib.error
import urllib.parse
from datetime import datetime
from pathlib import Path

# Load .env file if it exists
_env_path = Path(__file__).parent / ".env"
if _env_path.exists():
    for _line in _env_path.read_text().splitlines():
        _line = _line.strip()
        if _line and not _line.startswith("#") and "=" in _line:
            _key, _val = _line.split("=", 1)
            _val = _val.strip().strip('"').strip("'")
            os.environ.setdefault(_key.strip(), _val)

# ─────────────────────────────────────────────────────────────────
# CONFIGURE THESE via .env file or setup_agent.sh
SLACK_BOT_TOKEN = os.getenv("SLACK_BOT_TOKEN", "")
SLACK_CHANNEL_ID = os.getenv("SLACK_CHANNEL_ID", "")
AGENT_NAME = os.getenv("AGENT_NAME", "")
MESSAGES_TO_FETCH = 10
# ─────────────────────────────────────────────────────────────────

SLACK_API = "https://slack.com/api"

# Cache for user ID -> display name lookups
_user_cache = {}


def _git(cmd):
    """Run a git command and return stripped stdout, or empty string on failure."""
    try:
        result = subprocess.run(
            ["git"] + cmd.split(),
            capture_output=True, text=True, timeout=5,
            cwd=Path(__file__).parent
        )
        return result.stdout.strip() if result.returncode == 0 else ""
    except Exception:
        return ""


def git_context():
    """Gather current git state for message context."""
    return {
        "branch": _git("rev-parse --abbrev-ref HEAD"),
        "commit": _git("log -1 --format=%h"),
        "commit_msg": _git("log -1 --format=%s"),
        "user": _git("config user.name"),
    }


def slack_request(endpoint, method="GET", params=None, payload=None):
    """Make a Slack API request with one retry on transient failure."""
    for attempt in range(2):
        try:
            if method == "GET":
                query = urllib.parse.urlencode(params or {})
                url = f"{SLACK_API}/{endpoint}?{query}"
                req = urllib.request.Request(url, headers={
                    "Authorization": f"Bearer {SLACK_BOT_TOKEN}",
                })
            else:
                url = f"{SLACK_API}/{endpoint}"
                req = urllib.request.Request(
                    url,
                    data=json.dumps(payload or {}).encode("utf-8"),
                    headers={
                        "Authorization": f"Bearer {SLACK_BOT_TOKEN}",
                        "Content-Type": "application/json",
                    }
                )
            with urllib.request.urlopen(req, timeout=10) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError) as e:
            if attempt == 0:
                time.sleep(1)
                continue
            print(f"[slack_agent] Network error after retry: {e}", file=sys.stderr)
            return {"ok": False, "error": str(e)}


def resolve_user(user_id):
    """Resolve a Slack user ID to display name (cached)."""
    if user_id in _user_cache:
        return _user_cache[user_id]
    result = slack_request("users.info", params={"user": user_id})
    if result and result.get("ok"):
        user = result["user"]
        name = user.get("real_name") or user.get("profile", {}).get("display_name") or user.get("name", user_id)
    else:
        name = user_id
    _user_cache[user_id] = name
    return name


def check_config():
    errors = []
    if not SLACK_BOT_TOKEN or "YOUR" in SLACK_BOT_TOKEN:
        errors.append("SLACK_BOT_TOKEN not set")
    if not SLACK_CHANNEL_ID or "YOUR" in SLACK_CHANNEL_ID:
        errors.append("SLACK_CHANNEL_ID not set")
    if errors:
        print("[slack_agent] ERROR: Missing configuration:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        print("\nRun: bash setup_agent.sh", file=sys.stderr)
        sys.exit(1)


# ── READ ──────────────────────────────────────────────────────────

def cmd_read(count):
    check_config()
    print(f"\n{'='*55}")
    print(f"  SLACK — Last {count} messages from team channel")
    print(f"{'='*55}\n")

    result = slack_request("conversations.history", params={
        "channel": SLACK_CHANNEL_ID,
        "limit": count,
    })

    if not result or not result.get("ok"):
        err = result.get("error", "unknown") if result else "no response"
        print(f"[slack_agent] Slack API error: {err}", file=sys.stderr)
        return

    messages = result.get("messages", [])
    if not messages:
        print("  No messages yet in this channel.\n")
        return

    for msg in reversed(messages):
        ts = float(msg.get("ts", 0))
        time_str = datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M")

        if msg.get("bot_profile"):
            sender = msg["bot_profile"].get("name", "bot")
        elif msg.get("user"):
            sender = resolve_user(msg["user"])
        else:
            sender = msg.get("username", "unknown")

        text = msg.get("text", "").strip()
        print(f"[{time_str}] {sender}:")
        for line in text.split("\n"):
            print(f"  {line}")
        print()

    print(f"{'='*55}")
    print("  You are now up to date. Proceed with your session.")
    print(f"{'='*55}\n")


# ── POST ──────────────────────────────────────────────────────────

def cmd_post(message, final):
    check_config()
    ctx = git_context()
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    name = AGENT_NAME or ctx.get("user") or "Agent"
    label = "SESSION END" if final else "MID-SESSION UPDATE"
    emoji = ":white_check_mark:" if final else ":arrows_counterclockwise:"

    branch_info = ""
    if ctx["branch"]:
        branch_info = f"\n  Branch: `{ctx['branch']}`"
        if ctx["commit"]:
            branch_info += f" @ `{ctx['commit']}`"
        if ctx["commit_msg"]:
            branch_info += f" — {ctx['commit_msg']}"

    payload = {
        "channel": SLACK_CHANNEL_ID,
        "blocks": [
            {
                "type": "section",
                "text": {
                    "type": "mrkdwn",
                    "text": f"{emoji} *[{label}]* — *{name}* — `{now}`{branch_info}\n\n{message}"
                }
            },
            {"type": "divider"}
        ]
    }

    result = slack_request("chat.postMessage", method="POST", payload=payload)

    if result and result.get("ok"):
        print(f"[slack_agent] Posted to Slack: {label}")
    else:
        err = result.get("error", "unknown") if result else "no response"
        print(f"[slack_agent] Slack API error: {err}", file=sys.stderr)


# ── STATUS ────────────────────────────────────────────────────────

def cmd_status():
    check_config()
    ctx = git_context()
    now = datetime.now().strftime("%H:%M")
    name = AGENT_NAME or ctx.get("user") or "Agent"

    text = f":large_blue_circle: *{name}* is active — `{now}`"
    if ctx["branch"]:
        text += f"\n  Branch: `{ctx['branch']}`"
        if ctx["commit"]:
            text += f" @ `{ctx['commit']}`"

    result = slack_request("chat.postMessage", method="POST", payload={
        "channel": SLACK_CHANNEL_ID,
        "text": text,
    })

    if result and result.get("ok"):
        print(f"[slack_agent] Status heartbeat posted")
    else:
        err = result.get("error", "unknown") if result else "no response"
        print(f"[slack_agent] Slack API error: {err}", file=sys.stderr)


# ── AUTO (called by Claude Code hooks) ────────────────────────────

THROTTLE_FILE = Path(__file__).parent / ".slack_last_activity"
ACTIVITY_THROTTLE_SEC = 900  # 15 minutes between activity updates


def _should_throttle():
    """Return True if an activity update was posted recently."""
    try:
        if THROTTLE_FILE.exists():
            last = float(THROTTLE_FILE.read_text().strip())
            return (time.time() - last) < ACTIVITY_THROTTLE_SEC
    except Exception:
        pass
    return False


def _mark_posted():
    """Record that we just posted an activity update."""
    try:
        THROTTLE_FILE.write_text(str(time.time()))
    except Exception:
        pass


def cmd_auto(event, prompt=None):
    """Called automatically by SessionStart/SessionEnd/UserPromptSubmit hooks."""
    if not SLACK_BOT_TOKEN or "YOUR" in SLACK_BOT_TOKEN:
        return
    if not SLACK_CHANNEL_ID or "YOUR" in SLACK_CHANNEL_ID:
        return

    ctx = git_context()
    name = AGENT_NAME or ctx.get("user") or "Agent"
    now = datetime.now().strftime("%Y-%m-%d %H:%M")

    if event == "session-start":
        # Read recent messages so the agent sees what teammates posted
        result = slack_request("conversations.history", params={
            "channel": SLACK_CHANNEL_ID,
            "limit": 5,
        })
        if result and result.get("ok"):
            messages = result.get("messages", [])
            if messages:
                print(f"\n{'='*55}")
                print(f"  SLACK — Recent team activity")
                print(f"{'='*55}\n")
                for msg in reversed(messages):
                    ts = float(msg.get("ts", 0))
                    time_str = datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M")
                    if msg.get("bot_profile"):
                        sender = msg["bot_profile"].get("name", "bot")
                    elif msg.get("user"):
                        sender = resolve_user(msg["user"])
                    else:
                        sender = msg.get("username", "unknown")
                    text_line = msg.get("text", "").strip()
                    print(f"[{time_str}] {sender}:")
                    for line in text_line.split("\n"):
                        print(f"  {line}")
                    print()
                print(f"{'='*55}\n")

        text = f":rocket: *{name}* started a session — `{now}`"
        if ctx["branch"]:
            text += f"\n  Branch: `{ctx['branch']}`"
            if ctx["commit"]:
                text += f" @ `{ctx['commit']}` — {ctx.get('commit_msg', '')}"

    elif event == "session-end":
        text = f":checkered_flag: *{name}* ended session — `{now}`"
        if ctx["branch"]:
            text += f"\n  Branch: `{ctx['branch']}`"
            if ctx["commit"]:
                text += f" @ `{ctx['commit']}` — {ctx.get('commit_msg', '')}"
        # clean up throttle file on session end
        try:
            THROTTLE_FILE.unlink(missing_ok=True)
        except Exception:
            pass

    elif event == "activity":
        if _should_throttle():
            return  # posted recently, skip
        if not prompt:
            return
        # Truncate long prompts
        summary = prompt.strip()
        if len(summary) > 200:
            summary = summary[:200] + "..."
        text = f":hammer_and_wrench: *{name}* is working on — `{now}`"
        if ctx["branch"]:
            text += f" (`{ctx['branch']}`)"
        text += f"\n> {summary}"
        _mark_posted()

    else:
        return

    slack_request("chat.postMessage", method="POST", payload={
        "channel": SLACK_CHANNEL_ID,
        "text": text,
    })


# ── MAIN ──────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Agent Slack Bridge")
    sub = parser.add_subparsers(dest="command", required=True)

    r = sub.add_parser("read", help="Read recent Slack messages")
    r.add_argument("--count", type=int, default=MESSAGES_TO_FETCH,
                   help=f"Number of messages to fetch (default: {MESSAGES_TO_FETCH})")

    p = sub.add_parser("post", help="Post a message to Slack")
    p.add_argument("--message", required=True, help="The message to post")
    p.add_argument("--final", action="store_true",
                   help="Mark as end-of-session summary")

    sub.add_parser("status", help="Post a working-on heartbeat")

    a = sub.add_parser("auto", help="Auto-post (called by hooks)")
    a.add_argument("event", choices=["session-start", "session-end", "activity"],
                   help="Hook event type")
    a.add_argument("--prompt", default=None,
                   help="User prompt text (for activity updates)")

    args = parser.parse_args()

    if args.command == "read":
        cmd_read(args.count)
    elif args.command == "post":
        cmd_post(args.message, args.final)
    elif args.command == "status":
        cmd_status()
    elif args.command == "auto":
        cmd_auto(args.event, getattr(args, "prompt", None))


if __name__ == "__main__":
    main()
