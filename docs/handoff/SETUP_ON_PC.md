# Continuing on PC — setup instructions

This folder packages everything needed to continue the laptop session
(2026-05-07, session ID `79610daf-47c4-4b65-8f87-6ada8b7fecc8`) on a
different machine. The `memory/` directory contains the auto-memory
entries Claude built up during the session; `session-79610daf.jsonl`
is the full transcript.

## On the laptop (before switching)

You'll see uncommitted work in `git status`. To make it travel:

```powershell
git add -A
git commit -m "handoff: heading fix + MiDaS counters + map-matching plan + handoff bundle"
git push origin morad
```

(Or stash + push the stash as a branch if you don't want a commit yet.)

## On the PC (after switching)

### 1. Pull the branch

```powershell
cd C:\Users\morad\AndroidStudioProjects\NavSight1
git fetch origin
git checkout morad
git pull --ff-only
```

If the PC has the project at a different path or a different Windows
username, the Claude project folder name changes — see "Different path?"
below.

### 2. Restore Claude's memory + transcript

The auto-memory and session transcript live OUTSIDE the repo by default
(at `C:\Users\<user>\.claude\projects\<encoded-project-path>\`). Run
this PowerShell from the project root to copy them into place:

```powershell
$ProjectRoot = 'C:\Users\morad\AndroidStudioProjects\NavSight1'
$ProjectKey  = ($ProjectRoot -replace '\\', '-' -replace ':', '-')   # → "C--Users-morad-AndroidStudioProjects-NavSight1"
$ClaudeDir   = "$env:USERPROFILE\.claude\projects\$ProjectKey"

# Memory
New-Item -ItemType Directory -Force -Path "$ClaudeDir\memory" | Out-Null
Copy-Item "$ProjectRoot\docs\handoff\memory\*.md" "$ClaudeDir\memory\" -Force

# Transcript (lets `claude --resume` pick up where laptop left off)
Copy-Item "$ProjectRoot\docs\handoff\session-79610daf.jsonl" "$ClaudeDir\79610daf-47c4-4b65-8f87-6ada8b7fecc8.jsonl" -Force

Write-Host "Restored memory + transcript into $ClaudeDir"
```

### 3. Resume the session

Two options:

**Option A — try `--resume` (cross-machine resume is unsupported but
sometimes works if the transcript is in place):**

```powershell
claude --resume 79610daf-47c4-4b65-8f87-6ada8b7fecc8
```

**Option B — start a fresh Claude session and paste the continuation
prompt** from `docs/handoff/CONTINUATION_PROMPT.md`. This is the
reliable path. The new session will read the memory files (which the
restore step put in place) and the project's `IMPLEMENTATION_STATUS.md`
and have everything it needs.

### 4. Clean up the handoff bundle (optional)

Once the PC session is running and you've confirmed memory is restored,
the 4.7 MB transcript copy in the repo is no longer needed:

```powershell
git rm docs/handoff/session-79610daf.jsonl
git commit -m "handoff: drop session transcript after PC restore"
```

You can keep `docs/handoff/memory/` in the repo as a backup, or remove
that too if it's redundant with the live memory at `~/.claude/projects/...`.

## Different path on PC?

If the project lives at a different path or your Windows username is
different on the PC, the Claude project folder name changes. Adjust
the `$ProjectRoot` line in step 2 to match the PC's actual path. The
script derives the encoded folder name automatically.

The memory files themselves are path-independent — they reference
project files by relative path (e.g. `app/src/main/cpp/EKFState.cpp`),
so they work as long as the repo is at the path `$ProjectRoot` points
at on the PC.
