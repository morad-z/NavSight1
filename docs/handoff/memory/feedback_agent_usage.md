---
name: Parallel agent usage policy
description: Spawn role-assigned agents in parallel in a single message; never sequentially when independent
type: feedback
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
For every non-trivial task, spawn multiple parallel agents in a single message. Assign by role:
- C++ implementation → `cpp-reviewer` + `coder`
- Test writing → `tdd-guide`
- Sim analysis → `code-analyzer` or `python-reviewer`
- Architecture decisions → `architect`
- Security/correctness review after code changes → `security-reviewer` + `code-reviewer` in parallel

Never do sequential agent work when tasks are independent.

**Why:** Morad's standing instruction; matches the global `~/.claude/rules/common/agents.md` policy.

**How to apply:** Use a single message with multiple Agent tool calls. Wait for results, don't poll. Trivial single-file lookups still go through direct tools (Read/Grep/Glob).
