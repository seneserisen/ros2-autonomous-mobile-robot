# FaultNav Copilot Instructions

- Read `AGENTS.md` and the relevant files under `docs/` before substantial changes.
- Use `docs/project_context.md` for scope, `docs/architecture.md` for system boundaries, and `docs/testing.md` for exact verification commands.
- Keep changes limited to one coherent task and preserve unrelated work.
- Proceed with reversible local work. Require approval for destructive, production, paid, external, credentialed, privacy-sensitive, breaking-interface, or physical-hardware actions.
- Preserve separation between ground truth, measurements, estimates, and ROS middleware.
- State units, coordinate frames, timestamp assumptions, and random-seed behavior for affected interfaces.
- Do not invent APIs, packages, requirements, files, metrics, benchmarks, or successful checks.
- Verify new dependencies proportionately to risk.
- Add or update behavior-focused tests for meaningful changes, including relevant failure cases.
- Do not remove valid tests or weaken validation to make checks pass.
- Run the exact commands documented in `docs/testing.md` and report only checks that actually ran.
- Review the complete diff for regressions, secrets, debug artifacts, placeholders, unrelated edits, and unsupported claims.
- Update documentation and `docs/project_status.md` when behavior or verification status changes.
