# `docs/handoff/` — cross-machine session handoff bundle

Created 2026-05-07 to package the laptop Claude Code session
`79610daf-47c4-4b65-8f87-6ada8b7fecc8` for continuation on a different
machine without USB / OneDrive / direct file copy.

## Contents

- **`SETUP_ON_PC.md`** — step-by-step setup script for the PC,
  including the PowerShell snippet that restores Claude's auto-memory
  and (optionally) the session transcript to their canonical user-home
  locations.
- **`CONTINUATION_PROMPT.md`** — a self-contained prompt to paste into
  a fresh Claude Code session on the PC if `claude --resume` doesn't
  work. Points the new session at this folder + project docs and
  states the current state and next action.
- **`memory/`** — copies of the auto-memory entries from
  `~/.claude/projects/.../memory/`. 12 files. Index is `MEMORY.md`.
  Restore script copies these to the PC's user-home Claude memory dir.
- **`session-79610daf.jsonl`** — full session transcript (4.7 MB).
  Optional; only needed if `claude --resume` is to be tried on the PC.
  Can be removed from the repo once the PC session is running cleanly.

## How to use

1. On laptop: commit + push (your call when to do this — see plan rule
   "do not commit unless I say commit").
2. On PC: `git pull`, then run the PowerShell block in `SETUP_ON_PC.md`.
3. Either `claude --resume 79610daf-47c4-4b65-8f87-6ada8b7fecc8` or
   start fresh and paste `CONTINUATION_PROMPT.md`.

## Cleanup

Once the PC session is up and you've confirmed it has the right
context (memory loaded, IMPLEMENTATION_STATUS.md read), you can
remove the heavyweight handoff artifacts from the repo:

```powershell
git rm docs/handoff/session-79610daf.jsonl
# Optionally also:
# git rm -r docs/handoff/memory
git commit -m "handoff: drop transcript copy after PC continuation"
```

The `SETUP_ON_PC.md` and `CONTINUATION_PROMPT.md` files are small and
worth keeping for the next time this kind of handoff is needed.
