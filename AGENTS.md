# Repository Working Rules

## Scope

This repository contains preparation work for the 2026 electronics design competition, currently focused on the H-problem vehicle-mounted ball-balancing system. Keep chassis, vision, EdgeTalk control, shared protocols, and experiments in their documented ownership boundaries.

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
