RT-NAVI
=======

[![rust](https://github.com/rtk-rs/rt-navi/actions/workflows/rust.yml/badge.svg)](https://github.com/rtk-rs/rt-navi/actions/workflows/rust.yml)
[![crates.io](https://img.shields.io/crates/v/rt-navi.svg)](https://crates.io/crates/rt-navi)
[![crates.io](https://img.shields.io/crates/d/rt-navi.svg)](https://crates.io/crates/rt-navi)

[![License](https://img.shields.io/badge/license-MPL_2.0-orange?style=for-the-badge&logo=mozilla)](https://github.com/rtk-rs/binex/blob/main/LICENSE)

`rt-navi` is our real-time navigation application. It proposes interfaces to our 
[GNSS-RTK solver](https://github.com/rtk-rs/gnss-rtk) and supports one U-Blox receiver,
since Rust language offers great support for these devices.

If support for other devices appear, it should be easy to propose other interfaces.

`rt-navi` expects a single serial interface and currently does not manage several receivers.

Licensing
=========

This application is part of the [RTK-rs framework](https://github.com/rtk-rs) which
is delivered under the [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.

PPP / RTK 
=========

`rt-navi` offers direct/absolute navigation by default (also referred to as `PPP`).  

An `RTK` option will be developped in near future.

Compilation
===========

The U-Blox library (one of our dependency) is not compatible with `--all-features`, as we use
compilation features to select the U-Blox protocol (read down below). So `rt-navi` is not compatible
with this compilation mode, you have to select the features you are interested one by one.

Cross-compilation
=================

`rt-navi` is compatible with cross-compilation, but it currently requires support of the `std` library.

Deployment
==========

Activate `$RUST_LOG` to have more information. Use `trace` to unlock all traces.

Use `-p` to specify the serial interface to the U-Blox. 

```bash
RUST_LOG=debug rt-navi -p /dev/ttyACM0

[2025-05-11T09:31:21Z INFO  anise::almanac] Loading bytes as DAF/SPK
[2025-05-11T09:31:21Z DEBUG rt_navi] deployed with User {
        profile: Some(
            Pedestrian,
        ),
        clock_sigma_s: 0.001,
    } 

[...]
[2025-05-11T09:31:21Z DEBUG rt_navi] deployed with Config {
        timescale: GPST,
        method: SPP,
        user: User {
            profile: None,
            clock_sigma_s: 0.001,
        },

[...]
[2025-05-11T09:31:21Z INFO  rt_navi] solver deployed
[2025-05-11T09:31:21Z DEBUG rt_navi] deploying ublox..
[2025-05-11T09:31:21Z DEBUG rt_navi::ublox] UBX-RXM-RAWX
[2025-05-11T09:31:21Z DEBUG rt_navi::ublox] UBX-NAV-PVT
[2025-05-11T09:31:21Z DEBUG rt_navi::ublox] UBX-RXM-SFRBX
[2025-05-11T09:31:21Z DEBUG rt_navi::ublox] UBX-RXM-RATE
[2025-05-11T09:31:21Z DEBUG rt_navi::ublox] UBX-MON-VER
[2025-05-11T09:31:21Z DEBUG rt_navi::ublox] ublox: hw-version: 00080000, sw-version: EXT CORE 3.01 (111141) , extensions: ["ROM BASE 2.01 (75331)", "FWVER=TIM 1.10", "PROTVER=22.00", "MOD=NEO-M8T-0", "FIS=0xEF4015 (100111)", "GPS;GLO;GAL;BDS", "SBAS;IMES;QZSS"]
```

Soon the navigation messages gathering starts, and so does the measurement collection.  
When both align and everything becomes feasible, the solver naturally consumes all of this data and we obtain a P.V.T:

U-Blox specificity
==================

`rt-navi` currently requires a U-Blox receiver to operate. We offer several compilation options
to select the U-Blox protocol (`"protocol"` version being very important, because it actually
defines which receiver you can operate!):

- `ubx_proto23` v23 (Default, for M8 series)
- `ubx_proto14` v14
- `ubx_proto27` v27 (for F9 series)
- `ubx_proto31` v31 (for newer series)

P.V.T solutions
===============

TODO

Initial fix
===========

We offer a few options to specify initial `x`, `y` and `z` coordiates. This reduces the constraint
to initial fix and make it easier to obtain. We recommend using this only if you have a rather good idea.

```bash
rt-navi -p /dev/ttyACM0 --coords-ecef-km=1000.0,2000.0,3000.0
```

Ecosystem
=========

`rt-navi` uses the combination of several major crates to achieve its objectives

- [ANISE](https://github.com/nyx-space/anise)
- [Hifitime](https://github.com/nyx-space/hifitime)
- [Nyx-Space](https://github.com/nyx-space/nyx)
- [SP3](https://github.com/rtk-rs/sp3)
- [GNSS-RTK](https://github.com/rtk-rs/gnss-rtk)
- [UBlox](https://github.com/ublox-rs/ublox)
