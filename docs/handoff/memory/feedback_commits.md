---
name: Commit only on explicit "commit"
description: Never commit changes unless Morad's current message contains the word "commit"
type: feedback
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
Do not commit unless Morad explicitly says "commit" in that specific message. Edits, analysis, and code writes are fine; the boundary is `git commit`.

**Why:** Morad explicitly asked for this in the resume prompt. Avoids drift between sim acceptance verdicts and committed state.

**How to apply:** When work is complete, leave changes uncommitted and report. Wait for "commit" verbatim before running `git add`/`git commit`.
