# Security Policy

## Reporting a vulnerability

Do not publish credentials, personal data, confidential files, private research inputs, or detailed exploit instructions in a public issue.

Report sensitive findings privately to the repository owner through an approved private channel. Include:

- affected commit, branch, or release;
- affected component;
- reproduction conditions;
- likely impact;
- suggested mitigation, if known.

If a credential was exposed, revoke and rotate it immediately. Removing it from the latest commit does not remove it from Git history.

## Supported versions

FaultNav is active portfolio software and does not currently promise production security support or response times.

| Version | Support status |
|---|---|
| Current `main` branch | Best effort |
| Older development snapshots | Not supported |

## Scope

Relevant reports include:

- unsafe file or path handling;
- command or code execution;
- secret exposure;
- malicious configuration or artifact parsing;
- dependency or supply-chain issues;
- ROS 2 message or network trust problems;
- behavior that could become unsafe during future hardware integration;
- misleading integrity of generated metrics or engineering evidence.

Third-party platform or dependency vulnerabilities should also be reported to the relevant provider.

## Safety boundary

The repository is not a safety-certified controller. No physical-hardware command, interlock change, or safety-limit reduction should be performed by an automated agent without explicit owner approval and supervised validation.
