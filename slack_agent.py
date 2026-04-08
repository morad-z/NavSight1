#!/usr/bin/env python3
"""
Agent Slack Bridge — Read and post to your team's Slack channel.

Commands:
  python3 slack_agent.py read                          # Read last messages (session start / mid-session check)
  python3 slack_agent.py post --message "..."          # Post mid-session update
  python3 slack_agent.py post --message "..." --final  # Post end-of-session summary
"""

import argparse
import json
import os
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
            os.environ.setdefault(_key.strip(), _val.strip())

# ─────────────────────────────────────────────────────────────────
# CONFIGURE THESE (or set as environment variables)
SLACK_BOT_TOKEN = os.getenv("SLACK_BOT_TOKEN", "xoxb-YOUR-BOT-TOKEN-HERE")
SLACK_CHANNEL_ID = os.getenv("SLACK_CHANNEL_ID", "C0YOUR-CHANNEL-ID")
AGENT_NAME = os.getenv("AGENT_NAME", "Tameer")   # Change to your name
MESSAGES_TO_FETCH = 10                            # How many recent messages to read
# ─────────────────────────────────────────────────────────────────

SLACK_API = "https://slack.com/api"


def slack_get(endpoint: str, params: dict) -> dict:
    query = urllib.parse.urlencode(params)
    url = f"{SLACK_API}/{endpoint}?{query}"
    req = urllib.request.Request(url, headers={
        "Authorization": f"Bearer {SLACK_BOT_TOKEN}",
        "Content-Type": "application/json"
    })
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def slack_post_json(endpoint: str, payload: dict) -> dict:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{SLACK_API}/{endpoint}",
        data=data,
        headers={
            "Authorization": f"Bearer {SLACK_BOT_TOKEN}",
            "Content-Type": "application/json"
        }
    )
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def check_config():
    errors = []
    if SLACK_BOT_TOKEN.startswith("xoxb-YOUR"):
        errors.append("SLACK_BOT_TOKEN not set")
    if SLACK_CHANNEL_ID.startswith("C0YOUR"):
        errors.append("SLACK_CHANNEL_ID not set")
    if errors:
        print("[slack_agent] ERROR: Missing configuration:")
        for e in errors:
            print(f"  - {e}")
        print("\nSet them in .env or directly in slack_agent.py")
        exit(1)


# ── READ ──────────────────────────────────────────────────────────

def cmd_read(count: int):
    check_config()
    print(f"\n{'='*55}")
    print(f"  SLACK — Last {count} messages from team channel")
    print(f"{'='*55}\n")

    result = slack_get("conversations.history", {
        "channel": SLACK_CHANNEL_ID,
        "limit": count
    })

    if not result.get("ok"):
        print(f"[slack_agent] Slack API error: {result.get('error')}")
        return

    messages = result.get("messages", [])
    if not messages:
        print("  No messages yet in this channel.\n")
        return

    # Messages come newest-first, reverse to show oldest first
    for msg in reversed(messages):
        ts = float(msg.get("ts", 0))
        time_str = datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M")
        sender = msg.get("username") or msg.get("bot_profile", {}).get("name") or "unknown"
        text = msg.get("text", "").strip()
        print(f"[{time_str}] {sender}:")
        print(f"  {text}\n")

    print(f"{'='*55}")
    print("  You are now up to date. Proceed with your session.")
    print(f"{'='*55}\n")


# ── POST ──────────────────────────────────────────────────────────

def cmd_post(message: str, final: bool):
    check_config()
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    label = "SESSION END" if final else "MID-SESSION UPDATE"
    emoji = "✅" if final else "🔄"

    payload = {
        "channel": SLACK_CHANNEL_ID,
        "blocks": [
            {
                "type": "section",
                "text": {
                    "type": "mrkdwn",
                    "text": f"{emoji} *[{label}]* — *{AGENT_NAME}* — `{now}`\n{message}"
                }
            },
            {"type": "divider"}
        ]
    }

    result = slack_post_json("chat.postMessage", payload)

    if result.get("ok"):
        print(f"[slack_agent] Posted to Slack: {label}")
    else:
        print(f"[slack_agent] Slack API error: {result.get('error')}")


# ── MAIN ──────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Agent Slack Bridge")
    sub = parser.add_subparsers(dest="command", required=True)

    # read command
    r = sub.add_parser("read", help="Read recent Slack messages")
    r.add_argument("--count", type=int, default=MESSAGES_TO_FETCH,
                   help=f"Number of messages to fetch (default: {MESSAGES_TO_FETCH})")

    # post command
    p = sub.add_parser("post", help="Post a message to Slack")
    p.add_argument("--message", required=True, help="The message to post")
    p.add_argument("--final", action="store_true",
                   help="Mark as end-of-session summary (vs mid-session update)")

    args = parser.parse_args()

    if args.command == "read":
        cmd_read(args.count)
    elif args.command == "post":
        cmd_post(args.message, args.final)


if __name__ == "__main__":
    main()
