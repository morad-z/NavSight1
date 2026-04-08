# AgentSync — Slack Integration for AI Agents

## Quick Setup

```bash
# Option 1: Run setup script
bash setup_agent.sh

# Option 2: Create .env manually in repo root
SLACK_BOT_TOKEN=xoxb-10866349482854-10862074750835-YrYzTRuHysjW96r83ZKuI21y
SLACK_CHANNEL_ID=C0ARN08DUDA
AGENT_NAME=YourName
```

## What Happens Automatically (via Claude Code hooks)

| Event | What posts to Slack | Trigger |
|---|---|---|
| Session start | Reads last 5 messages (printed to agent context) + posts ":rocket: started session" | `SessionStart` hook |
| Activity update | ":hammer: *Name* is working on — user's prompt" | `UserPromptSubmit` hook (throttled: max 1 per 15 min) |
| Session end | ":checkered_flag: *Name* ended session" + branch/commit | `SessionEnd` hook |

All automatic posts include git branch, commit hash, and commit message.
If `.env` is missing, hooks silently skip — no errors, no crash.

## What You (the AI agent) Should Do

### On session start:
The `SessionStart` hook automatically reads the last 5 Slack messages and prints them into your context, then posts that you started a session. You will see teammate updates without running any command.

If you need more history, run manually:
```bash
python slack_agent.py read --count 20
```

### During the session:
Activity updates are automatic. For important milestones, post manually:
```bash
python slack_agent.py post --message "Fixed BUG-014: scale calibration now works"
```

### On session end:
Post a summary of what you did:
```bash
python slack_agent.py post --message "Done: fixed scale calibration, updated EKF. Next: ZUPT tuning" --final
```

## Manual Commands

```bash
python slack_agent.py read                          # Read last 10 messages
python slack_agent.py read --count 20               # Read last 20 messages
python slack_agent.py post --message "..."          # Mid-session update
python slack_agent.py post --message "..." --final  # End-of-session summary
python slack_agent.py status                        # Quick "I'm active" heartbeat
```

## Files

| File | Purpose |
|---|---|
| `slack_agent.py` | Main Slack bridge script |
| `scripts/slack_activity_hook.py` | Hook wrapper for UserPromptSubmit (reads prompt from stdin JSON) |
| `setup_agent.sh` | First-time setup (creates .env) |
| `.env` | Tokens + agent name (**gitignored — never commit**) |
| `.slack_last_activity` | Throttle state file (gitignored, auto-managed) |

## Rules

- **NEVER** hardcode, log, or echo the `SLACK_BOT_TOKEN`
- **NEVER** commit `.env` to git
- **ALWAYS** read Slack at session start to check for teammate updates
- **ALWAYS** post a `--final` summary at session end
