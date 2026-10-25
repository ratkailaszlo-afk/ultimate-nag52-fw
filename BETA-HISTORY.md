# Beta firmware history through SA14-G2

## Shared upstream base and history format

The custom firmware series is derived from the upstream Ultimate-NAG52 firmware, with the stable source commit [`1b96089660e97c91811b3d9cda6ca6f82b458c69`](https://github.com/rnd-ash/ultimate-nag52-fw/commit/1b96089660e97c91811b3d9cda6ca6f82b458c69) as its pinned base. The public branch keeps the upstream commit as an ancestor. Each named beta below has a Git tag and an exact frozen source tree.

The original development folders did not retain their own Git commit graph. The tags in this fork are publication-time snapshot imports made from the retained source trees. For CA1–CA3, no separate per-build source manifest tied each individual binary to a full source hash; the retained CA lineage and the later CA3-to-CA4 source comparison are the available provenance. From CA4 onward, the records more directly identify parent versions, source patches, or source manifests. Import commit dates are publication dates, not historical development dates.

**Binary assets:** each image is attached to the GitHub Release named in the tag column. SHA-256 is included so developers can identify the exact image. Files marked offline-only were not installed on the reference TCU.

| Tag / firmware | Developed from | What changed | Recorded status |
| --- | --- | --- | --- |
| [`beta-ca1`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-ca1) · `1b96089-ca1` | Upstream stable line | First retained C/A selector and custom profile branch. | Historical beta snapshot. |
| [`beta-ca2`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-ca2) · `1b96089-ca2` | CA1 | TCC profile behavior and its first dedicated profile changes. | Historical beta snapshot. |
| [`beta-ca3`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-ca3) · `1b96089-ca3` | CA2 | Drivetrain and shift-path fixes. | Historical beta snapshot. |
| [`beta-ca4`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-ca4) · `1b96089-ca4` | CA3 | TCC Open/Slipping distinction, diagnostic snapshots, timeout cleanup, map ownership, ADC bounds and bounded Zener timing. | Historical beta; the retained record describes a narrow upstream-main-based change. |
| [`beta-ca5`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-ca5) · `1b96089-ca5` | CA4 | Smoother C-mode 1→2 and 2→3 torque return, a small overlap-timing change, and a conditional C-mode lift-off 3→4 filter. | Installed and read back on the reference setup in the 2026-10-18 record. |
| [`beta-sa6`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa6) · `1b96089-sa6` | CA5 | S-mode city profile: direct warm map following, bounded cold correction, revised S downshift behavior, and S/A selector behavior. | Installed and read back on the reference setup. |
| [`beta-sa7-r1a`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa7-r1a) · `1b96089-sa7-r1a` | SA6 | Offline reliability fixes for zero-RPM torque math, map initialization, calibration reload validation and S downshift-map selection. | **Offline only; HOLD_NO_WRITE.** |
| [`beta-sa8-r1b`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa8-r1b) · `1b96089-sa8-r1b` | SA7-R1a | Carries the R1a fixes and adds producer-cycle TCC RAM trace for cycle-level observation. | Installed, read back and checked with a stationary USB logger trial. |
| [`beta-sa9-r2u`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa9-r2u) · `1b96089-sa9-r2u` | SA8-R1b; pinned upstream base above | Consolidates TCC state, signal freshness, shift-end cleanup and UART/VFS diagnostic output fixes. Retains the S/A behavior and existing map/default configuration. | Installed and read back on the reference setup. |
| [`beta-sa10-lbm1`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa10-lbm1) · `1b96089-sa10-lbm1` | SA9-R2u | LookupByteMap allocation and lookup-path reliability work. This is the development base for SA11. | **Offline candidate only; not deployed.** |
| [`beta-sa11-r1`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa11-r1) · `1b96089-sa11-r1` | SA10-LBM1 | Profile-map initialization checks, TCC/CAN state reporting, and additional safety checks around map and runtime failure paths. | Offline-built first, then installed and read back on the reference setup. Q1 was a separate map pack, not part of this binary. |
| [`beta-sa12-r1`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa12-r1) · `1b96089-sa12-r1` | SA11-R1 | Adds adaptation/input guards, sensor and CAN failure handling, and a read-only runtime monitor record. | **Offline candidate only; not deployed.** |
| [`beta-sa12-r2`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa12-r2) · `1b96089-sa12-r2` | SA12-R1 | Adds a narrowly gated third-gear coast TCC-open path while retaining existing profile and map defaults. | Installed and read back. Q2 was a separate, vehicle-specific map pack and is not included. |
| [`beta-sa13-r1-v3`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa13-r1-v3) · `1b96089-sa13-r1` | SA12-R2 | Adds kickdown press-edge handling, bounded USB receive recovery for partial frames, and guarded core NVS initialization. | Installed and read back. Earlier SA13 v1/v2 candidates are not represented as releases. |
| [`beta-sa14-g1`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa14-g1) · `1b96089-sa14-g1` | SA13-R1 v3 | First retained gliding-mode implementation with guarded TCC opening and post-shift/city-path changes. | **Offline-only and superseded by G2.** Its initial 80 km/h proposal was replaced in G2. |
| [`beta-sa14-g2`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/tag/beta-sa14-g2) · `1b96089-sa14-g2` | SA14-G1; ultimately the pinned upstream base above | Refines fill, RPM/slip feedback, handoff, interruption and fallback. Gliding is gated to D, actual/target fifth gear, strictly above 60 km/h, valid signals, automatic profile, and known-inactive cruise/limiter/ART conditions. It also bounds G3/G4 post-shift TCC recovery and adds a confirmed S/A 3→4 request. | Installed and read back. Stationary logger proof only; **active gliding and road comfort remain unverified**. |

## Exact firmware images

| Firmware | Release asset | Size | SHA-256 |
| --- | --- | ---: | --- |
| CA1 | [`firmware-1b96089-ca1.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-ca1/firmware-1b96089-ca1.bin) | 417,728 B | `7e4685c407b047b7dc716691a987bedd38e5cd97851938a7fbb889598bfb90f9` |
| CA2 | [`firmware-1b96089-ca2.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-ca2/firmware-1b96089-ca2.bin) | 417,728 B | `427249b6a3b5b87f5dda89926bcc10c49b4be14d344cafa4f12c28015b6ae7e7` |
| CA3 | [`firmware-1b96089-ca3.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-ca3/firmware-1b96089-ca3.bin) | 417,792 B | `fe518fc334dfa1d68e85975ebe64290e02272d5499b7b491ad95d21b646f2db9` |
| CA4 | [`firmware-1b96089-ca4.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-ca4/firmware-1b96089-ca4.bin) | 417,984 B | `f89ff10dab69491717dee47167115bc3d5e83fdc0fbb373ad36ff469cf5ff1eb` |
| CA5 | [`firmware-1b96089-ca5.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-ca5/firmware-1b96089-ca5.bin) | 418,848 B | `822dc35ed25bcc5467bc3cfeb645c439b457fbf59083d7fa38a1dccd9bd00033` |
| SA6 | [`firmware-1b96089-sa6.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa6/firmware-1b96089-sa6.bin) | 418,768 B | `e22f4f7cbc7a6c4ae103fc5d0473ad908cae59d438a1cf5b0ed4e6063f0d5760` |
| SA7-R1a | [`firmware-1b96089-sa7-r1a-offline.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa7-r1a/firmware-1b96089-sa7-r1a-offline.bin) | 421,232 B | `f4f86779bd4c374ca8f750a4ee97a332c5daa19f882f1ede47240e74c345748b` |
| SA8-R1b | [`firmware-1b96089-sa8-r1b.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa8-r1b/firmware-1b96089-sa8-r1b.bin) | 424,320 B | `4573aed5c82fad8c171947a47c9a4248cae3d811476c1c57f6bd82998f9eddbd` |
| SA9-R2u | [`firmware-1b96089-sa9-r2u.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa9-r2u/firmware-1b96089-sa9-r2u.bin) | 431,984 B | `585d61398265ea2f642ae88daf20d63132485d8808851e634f7c8ca6499f998a` |
| SA10-LBM1 | [`firmware-1b96089-sa10-lbm1-offline.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa10-lbm1/firmware-1b96089-sa10-lbm1-offline.bin) | 432,272 B | `24a4b26fb3590f8028f27208795e3aeb7272cd0f032e1905cd38940a8541fd9a` |
| SA11-R1 | [`firmware-1b96089-sa11-r1-offline.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa11-r1/firmware-1b96089-sa11-r1-offline.bin) | 432,112 B | `4338d91bbd4c890bc5bd2bae87e575cbaafb32a1e36308df9c2de2e8d3ad40a3` |
| SA12-R1 | [`firmware-1b96089-sa12-r1-offline.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa12-r1/firmware-1b96089-sa12-r1-offline.bin) | 436,928 B | `91c2da6a25d4d828c11764229765ca58c3488ccad9686de2f54a0faf6a7e40a3` |
| SA12-R2 | [`firmware-1b96089-sa12-r2-offline.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa12-r2/firmware-1b96089-sa12-r2-offline.bin) | 437,360 B | `778a4f8b4e37b0e2a7295016165149ad0427b6798c00b45bc42dc5eb736392d3` |
| SA13-R1 v3 | [`firmware-1b96089-sa13-r1-v3.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa13-r1-v3/firmware-1b96089-sa13-r1-v3.bin) | 437,584 B | `71d5963f3be30d34008a8ddd0267623878572871433bff49ae5e2c8e641c67b3` |
| SA14-G1 | [`firmware-1b96089-sa14-g1-superseded.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa14-g1/firmware-1b96089-sa14-g1-superseded.bin) | 443,904 B | `f381fb3dad527958bb0c18554d9ddd15742ed8098145f50bdd89d045d57b5f6d` |
| SA14-G2 | [`firmware-1b96089-sa14-g2.bin`](https://github.com/ratkailaszlo-afk/ultimate-nag52-fw/releases/download/beta-sa14-g2/firmware-1b96089-sa14-g2.bin) | 447,952 B | `555938e63eadbace2dd555731801ccc51f37ee742a673ef8776b6e35473b8545` |

## What the SA14-G2 features mean

### Guarded gliding mode

The controller can request the torque-converter clutch to open only when the actual and target gear are both fifth, selector state is D, speed is strictly above 60 km/h, the profile is automatic, the ATF is within the configured entry range, and required pedal, speed, ratio, brake, cruise and dynamics signals are valid and fresh. Cruise control, limiter and ART must be known inactive. Missing, stale or unsupported inputs block entry.

The mode is stateful: it checks entry conditions, opens the TCC, monitors slip, then performs a bounded fill and handoff when the driver asks for torque or another guard trips. Interruption, timeout and fallback states are exposed through a read-only 64-byte GLD2 record. A G display state is requested with existing F-indicator priority; actual instrument-cluster display support was not verified on the vehicle.

### Bounded recovery and city behavior

The reapply path uses a limited fill window, observes RPM slip and timing, and requires stable software conditions before reporting handoff success. A rate guard remains active during recovery. Timeout selects a limited normal-map fallback and is reported as a software outcome. The work proxy is not measured clutch energy, and `Contact Possible` is not a contact sensor reading.

For S/A driving, SA14-G2 also routes G3/G4 post-shift TCC recovery through bounded control and requires a confirmed 150 ms S/A 3→4 request with an RPM margin over the downshift map. These changes do not add a new permanent pressure-map offset or a new NVS/map schema.

### Limits of the evidence

The retained offline suite used native source tests, sanitizers and deterministic replay. The deployed SA14-G2 was read back, and its matching logger recorded a 20-second stationary USB session with GLD2 inactive. That evidence does not establish on-road gliding, instrument-cluster display, physical TCC contact, hydraulic pressure, clutch temperature, or improved shift comfort. Pressure and PWM values are commands or telemetry unless independently measured.
