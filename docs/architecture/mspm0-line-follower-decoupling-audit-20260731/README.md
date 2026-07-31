# MSPM0 line-follower decoupling audit

This directory records the 2026-07-31 architecture audit of a detached local
`wit-oled-hardware-spi` snapshot. It is preserved as design evidence and a
refactoring handoff, not as a statement that the current `prep/2026` firmware
still has every cited issue.

The remote repository already contained a newer MSPM0 implementation when this
audit was published. Therefore:

- source line references in these documents refer to the detached snapshot;
- no legacy firmware source was copied over the newer repository version;
- each recommendation must be checked against the current
  `firmware/mspm0/wit-oled-hardware-spi` tree before implementation.

## Snapshot fingerprints

| File | SHA-256 |
|---|---|
| `main.c` | `3d565559f44aae9f3a0d8dae9fdaabf3b0b4fc48c41f2a446e9ad7fd822a2f2f` |
| `Drivers/GRAY/track.c` | `029c1f7b3eabb9064bfae0d83c5009b5adad6a7ce97ef182d052771b2c066975` |
| `Drivers/MOTOR/motor.c` | `2467880b74b1b8c7add2082425bc5eb88cc2f521a427943b04c639c185ef082b` |
| `Drivers/PID/pid.c` | `375b1e0c3a229013c6c0cc2a978878dca954abd031c689fae49de18fe5d40050` |

## Contents

- `00-features.md`: current-state feature boundaries in the audited snapshot.
- `01-flowcharts/`: per-feature current-state control flows.
- `02-duplication-report.md`: duplicate responsibilities and ownership conflicts.
- `03-unified-proposal.md`: proposed minimal ownership-based architecture.
- `04-handoff-prompts.md`: staged implementation-planning prompts.
