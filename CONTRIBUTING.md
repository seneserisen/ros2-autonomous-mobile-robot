# Contributing to FaultNav ROS 2

## Before starting

1. Read `AGENTS.md`.
2. Read `docs/project_context.md`, `docs/architecture.md`, `docs/testing.md`, and `docs/project_status.md`.
3. Check existing issues and current limitations.
4. Define observable acceptance criteria before implementation.
5. Do not place secrets, personal data, confidential material, or restricted university content in issues, commits, logs, or artifacts.

## Development workflow

1. Create a focused branch.
2. Inspect related implementation and tests before editing.
3. Keep the change to one coherent task.
4. Preserve unrelated work.
5. Add or update behavior-focused tests.
6. Run the exact commands in `docs/testing.md`.
7. Review the complete diff.
8. Update documentation when behavior, interfaces, assumptions, metrics, or limitations change.

## Robotics and numerical changes

Document and verify relevant:

- units and sign conventions;
- coordinate frames and transform direction;
- timestamp and sampling assumptions;
- random seed and reproducibility behavior;
- ground-truth, measurement, and estimate separation;
- analytical or independent reference cases;
- simulation versus hardware boundaries.

Follow `docs/robotics_engineering_rules.md`.

## Commit guidance

Use clear, purpose-focused commits. Examples:

```text
feat: add EKF prediction model
fix: reject non-positive wheel geometry
 test: add gyro-bias innovation regression case
 docs: define covariance ordering and units
 chore: add repository quality instructions
```

Do not combine broad formatting, unrelated cleanup, dependency upgrades, and feature work unless they are inseparable and explained.

## Pull requests

Use the pull-request template and include:

- objective and acceptance criteria;
- exact commands and outcomes;
- manual checks;
- interface and compatibility effects;
- risks, assumptions, and remaining limitations;
- an accurate verification status.

Passing CI is necessary but does not prove ROS 2 runtime, simulator, hardware, security, or real-world accuracy.

## Hardware work

Do not send commands to physical hardware or weaken safety behavior without explicit owner approval and a separate supervised test plan.
