RT-NAVI
=======

`RT-NAVI` stands for Real Time Navigation and aims at real time high precision navigation.

Real time / Post processing
===========================

This application is dedicated to _real time_ navigation, as opposed to _post processed_ navigation.  
[Our position solver GNSS-RTK](https://github.com/rtk-rs/gnss-rtk) serves as the solution solver.  

We differentiate real time from post processed navigation, by capability to resolve a position at the instant
of signal sampling. The signal _Sampling Period_ (also refered to as _Observation Period_) dictates the rate
of solutions production.

[The Georust RINEX-Cli application](https://github.com/georust/rinex) is dedicated to post processed navigation
and does not require a hardware interface. It uses the same navigation core as this application.

GNSS receiver (Hardware)
========================

For this task, `rt-navi` requires real time access to a GNSS receiver.  
We currently support `Ublox 8` device, 9 and 10 series being hypothetically supported.

Cross-compilation
=================

`rt-navi` is compatible with cross-compilation, but it currently requires support of the `std` library.

Ecosystem
=========

`rt-navi` uses the combination of several major crates to achieve its objectives

- [ANISE](https://github.com/nyx-space/anise)
- [Hifitime](https://github.com/nyx-space/hifitime)
- [Nyx-Space](https://github.com/nyx-space/nyx)
- [SP3](https://github.com/georust/rinex/sp3)
- [UBlox](https://github.com/ublox-rs/ublox)
