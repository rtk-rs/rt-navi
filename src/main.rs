/*
 * rt-navi is part of the rtk-rs framework.
 *
 * Authors: Guillaume W. Bres <guillaume.bressaix@gmail.com> et al.
 * (cf. https://github.com/rtk-rs/rt-navi/graphs/contributors)
 * This framework is shipped under Mozilla Public V2 license.
 *
 * Documentation:
 *
 *   https://github.com/rtk-rs/
 *   https://github.com/rtk-rs/gnss-rtk
 *   https://github.com/ublox-rx/ublox
 */

mod bias;
mod cli;
mod clock;
mod kepler;
mod rtcm;
mod time;
mod ublox;

use env_logger::{Builder, Target};

#[macro_use]
extern crate log;

use thiserror::Error;

use gnss_rtk::prelude::{
    AbsoluteTime, Almanac, Bias, BiasRuntime, Config, Epoch, Frame, Method, Orbit, OrbitSource,
    TimeScale, User, EARTH_J2000, PPP, SV,
};

use tokio::sync::mpsc;

use ublox::{Message, Ublox};

use crate::{
    bias::BiasModels,
    cli::Cli,
    clock::{ClockBuffer, SvClock},
    kepler::{KeplerBuffer, SvKepler},
    time::Time,
};

#[derive(Debug, Error)]
pub enum Error {
    #[error("non supported constellation {0}")]
    NonSupportedGnss(u8),
    #[error("non supported signal {0}")]
    NonSupportedSignal(u8),
}

#[tokio::main]
async fn main() -> Result<(), Error> {
    let mut builder = Builder::from_default_env();

    builder
        .target(Target::Stdout)
        .format_timestamp_secs()
        .format_module_path(false)
        .init();

    // cli and user args
    let cli = Cli::new();

    let opts = cli.serial_opts();

    let almanac = Almanac::until_2035().unwrap_or_else(|e| {
        panic!("Failed to obtain Almanac definition: {}", e);
    });

    let frame = almanac.frame_from_uid(EARTH_J2000).unwrap_or_else(|e| {
        panic!("Failed to obtain Frame definition: {}", e);
    });

    let sampling_period_nanos = 10_000_000_000;

    // create channels
    let (ublox_tx, mut ublox_rx) = mpsc::channel(16);

    let bias = BiasModels {};
    let time_source = Time {};

    let user_profile = User::default();
    let cfg = Config::static_preset(Method::SPP);

    info!("deployed with {:#?} {:#?}", user_profile, cfg);

    let (x_km, y_km, z_km) = (4604.427, 373.312, 4383.065);
    let (x_m, y_m, z_m) = (x_km * 1.0e3, y_km * 1e3, z_km * 1e3);

    // let mut ppp = PPP::new_survey(almanac, frame, cfg, orbit_source, time_source, bias);
    let mut ppp = PPP::new(
        almanac,
        frame,
        cfg,
        eph_buffer,
        time_source,
        bias,
        Some((x_m, y_m, z_m)),
    );

    // deploy hardware
    let mut ublox = Ublox::new(opts, ublox_tx);

    ublox.init(sampling_period_nanos);

    tokio::spawn(async move {
        ublox.tasklet().await;
    });

    loop {
        while let Some(msg) = ublox_rx.recv().await {
            match msg {
                Message::Proposal(candidates) => {
                    let epoch = candidates[0].t;
                    debug!("{} - new proposal ({} candidates)", epoch, candidates.len());

                    match ppp.resolve(user_profile, epoch, &candidates) {
                        Ok(pvt) => {
                            let (lat, long, alt) = pvt.lat_long_alt_deg_deg_m;

                            info!("new solution | lat={:.6}° long={:.6} alt={:.6}m | dt={:.6E}s drift={:.6E}s/s", 
                                lat, long, alt, pvt.clock_offset_s ,pvt.clock_drift_s_s);
                        },
                        Err(e) => {
                            error!("ppp error: {}", e);
                        },
                    }
                },

                Message::SvClock(sv_clock) => {
                    eph_buffer.latch_clock(sv_clock);
                },

                Message::SvKepler(sv_kepler) => {
                    eph_buffer.latch_kepler(sv_kepler);
                },
            }
        }
    }
}
