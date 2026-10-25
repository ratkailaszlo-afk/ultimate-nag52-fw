# Ultimate-NAG52 community beta firmware

This is a personal development fork of [rnd-ash/ultimate-nag52-fw](https://github.com/rnd-ash/ultimate-nag52-fw). It preserves the upstream repository history and adds a documented series of custom beta firmware snapshots through **SA14-G2**.

## Source origin

The custom beta line is anchored to the upstream stable firmware commit [`1b96089660e97c91811b3d9cda6ca6f82b458c69`](https://github.com/rnd-ash/ultimate-nag52-fw/commit/1b96089660e97c91811b3d9cda6ca6f82b458c69), released on 2026-06-08. The public branch `beta/sa14-g2` starts from that exact commit. Each beta is available as a Git tag and as a matching firmware image in GitHub Releases. See [BETA-HISTORY.md](BETA-HISTORY.md) for the feature-by-feature lineage, deployment evidence, image sizes, and SHA-256 checksums.

The original work folders were frozen source snapshots rather than Git branches. The beta tags therefore use clearly named snapshot-import commits created for this publication. Those commits preserve the upstream commit as an ancestor and keep each retained source tree reviewable; they are not presented as the original historical commit timestamps.

This is an independent community fork. It is not an official upstream release, and its changes have not been accepted by the upstream maintainers.

## Current endpoint: SA14-G2

- Firmware ID: `1b96089-sa14-g2`
- Binary: **447,952 bytes**
- SHA-256: `555938e63eadbace2dd555731801ccc51f37ee742a673ef8776b6e35473b8545`
- Paired Windows logger: [ultimate-nag52-windows-logger](https://github.com/ratkailaszlo-afk/ultimate-nag52-windows-logger), version **0.4.8**
- SA14-G2 was installed and read back on one reference TCU. A 20-second stationary USB recording confirmed all 13 logger streams and 19 GLD2 records with no reported cycle gaps or decode errors. GLD2 was inactive during that test; it does not prove on-road gliding behavior, physical clutch contact, hydraulic pressure, or shift comfort.

SA14-G2 adds a guarded TCC-open gliding mode for stable fifth gear, strictly above 60 km/h, with required valid signals and driver-assistance states. It also improves bounded fill, slip feedback, handoff/recovery, post-shift TCC handling, and the automatic S/A 3-to-4 decision. Exact behavior and limits are in [BETA-HISTORY.md](BETA-HISTORY.md).

## What is in the firmware

- **Gearbox control:** shift profiles, shift request logic, transition and timeout handling.
- **Torque-converter control:** Open/Slipping/Closed state handling, pressure requests, prefill and recovery paths.
- **Signals and CAN:** engine, pedal, brake, speed, temperature, gear and selector validity and age checks.
- **Diagnostics and monitoring:** KWP services, runtime counters, TCC trace and the read-only GLD2 status record.
- **Memory and configuration:** map loading, calibration checks, NVS configuration and module interfaces.

Firmware pressure values are requests or software models. They are not measured hydraulic pressure. RPM, torque, contact, temperature and timing signals also have source and sampling limits described in the release history.

## Build from source

The recorded target build uses PlatformIO Core 6.1.18, `espressif32@6.8.1`, ESP-IDF 5.3.0 and the `unified` environment. With PlatformIO installed:

```powershell
python -m pip install platformio==6.1.18
python -m platformio run -e unified
```

This command builds the firmware image. Build output is toolchain-dependent; verify the resulting image checksum and version metadata before using it. The repository retains the upstream GPL-3.0 license and its contribution guidance. Initialize the Doxygen CSS submodule only when generating code documentation:

```powershell
git submodule update --init --recursive
```

## Scope and files intentionally excluded

This repository contains firmware source and the beta app images. It does not contain vehicle-specific map packs, NVS settings, calibration dumps, full-flash backups, ECU/TCU identifiers, or driving logs. Q1/Q2 map packs were separate from the firmware image and are not generic settings for other vehicles.

Check each release entry before using an image: SA7-R1a, SA10-LBM1, SA12-R1 and SA14-G1 are explicitly marked as offline-only or superseded. SA14-G2 is still a beta for a specific reference setup; its active gliding behavior has not been road-validated.
