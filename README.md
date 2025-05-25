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

RT-NAVI
=======

`rt-navi` standards for Real Time Navigation, and is counterpart workflow to our postprocessing
framework (basically [GNSS-Qc](https://github.com/rtk-rs/gnss-qc) and [RINEX-Cli](https://github.com/rtk-rs/rinex-cli)).

`rt-navi` is currently limited to PPP navigation (no reference station).
RTK navigation is not 100% feasible yet.

Compilation
===========

The U-Blox library (one of our dependency) is not compatible with `--all-features`, as we use
compilation features to select the U-Blox protocol (read down below). So `rt-navi` is not compatible
with this compilation mode, you have to select the features you are interested one by one.

Other crate features

- `rtcm`: unlocks NTRIP client, RTCM decoding and forwarding.
You cannot deploy real-time RTK without this option.

Cross-compilation
=================

`rt-navi` is compatible with cross-compilation, but it currently requires the `std` library,
due to several advanced internal dependencies.

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

PPP Deployment
==============

Deploying to `PPP` (direct / absolute) navigation is the simplest option. In this use case,
we will navigate without any ground-reference. This is how you deploy for example,
in remote areas without network access, or as a ground-reference yourself.

Connect to your U-Blox with `--port,-p` (serial interface):

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

RTK-Deployment
==============

`RTK` deployment requires connection to one Base station, this is currently only feasible
through an NTRIP birectionnal link. We only support one reference link as of today.

When selecting your reference station, you should select the one that reduces the baseline projection,
so only use reference stations within your local area.

`RTK` deployment is selected by specifying the remote server (no other option needed). 
In otherwords, this application is smart enough to change the navigation technique. 
But on the other hand, this is also a strong assumption, and we require that the link with the Base station
_remains alive at all times_. Any Epoch for which observations are not realized on both sites cannot generate
a solution.

Simply add the `--ntrip` option to  describe the NTRIP server. We support two formats,
in any case, the Host URL, port and mountpoint are always required:

1. password-less: `host:port/mountpoint`
2. with credentials: `host:port/mountpoint/user=$name,password=$password`

Example (1): 

```C
rt-navi -p /dev/ttyACM0 --ntrip 192.168.1.10:2101/custom
```

Example (2):

```C
rt-navi -p /dev/ttyACM0 \
    --ntrip caster.centipede.fr:2101/ENSMM/user=centipede,password=centipede
```

You can deploy a `RTCM` client for as many base stations as you need, using the `--rtcm` option.
A client (listener) is deployed for each one that you specify.   
NB: `gnss-rtk` is currently limited to one REF station is practice.

Example:

```C
RUST_LOG=debug rt-navi -p /dev/ttyACM0 --rtcm ntrip
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
