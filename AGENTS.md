# Repository Working Rules

## Scope

This repository contains preparation work for the 2026 electronics design competition, currently focused on the H-problem vehicle-mounted ball-balancing system. Keep chassis, vision, EdgeTalk control, shared protocols, and experiments in their documented ownership boundaries.

For Q1-Q6 work, follow `docs/architecture/mission-code-layout.md`. M33 owns the question-specific
phase machines; M55 runs task-independent estimation/control from an explicit mission context;
MSP and Pi expose generic executors/services and facts. Preserve the documented function-call
direction: algorithms never read mission globals or call transports/drivers, and only the M33
safety gate may reach the RS00 actuator adapter. Do not grow `main.c` beyond initialization and
runtime startup.

## Branches

- Work from `prep/2026` or a focused feature branch.
- Do not create, populate, or merge into `main` until the repository owner explicitly approves a stable baseline.
- Treat `PSOC_E84_robot` as a read-only reference repository, not as this repository's history or architecture.

## Safety

- Never enable motors or autonomous motion as part of an automated test.
- Hardware tests must state wheel/actuator state, power source and current limit, emergency stop, and operator takeover method.
- Treat configuration that can arm a vehicle or move an actuator as safety-critical.
- Do not store Wi-Fi credentials, private keys, device serial identifiers, or personal access tokens in Git.

## Documentation

- Record hardware model, firmware version, wiring, parameters, and verification result.
- Keep official sources separate from reference-repository evidence and community tutorials.
- Store architectural decisions in `docs/decisions/`.
- Put unverified prototypes in `experiments/`; promote them only after repeatable hardware tests.

## Files

- Do not commit large videos, installers, raw datasets, or full hardware logs.
- Prefer source URLs plus version and checksum for downloaded tools.
- Add or update a README before adding substantial code to a subsystem.
