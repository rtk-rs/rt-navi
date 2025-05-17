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

#[macro_use]
extern crate log;

mod bias;
mod cli;
mod rt_navi;
// mod clock;
mod ephemeris;
mod kepler;
// mod rtcm;
mod time;
mod ublox;
mod ui;

use env_logger::{Builder, Target};

use std::{
    io,
    time::{Duration, Instant},
};

use thiserror::Error;

use tokio::sync::mpsc;

use crate::{
    bias::BiasModels,
    cli::Cli,
    kepler::KeplerBuffer,
    rt_navi::RtNavi,
    time::Time,
    ublox::{Message, Ublox},
};

use gnss_rtk::prelude::{Almanac, Config, Method, Rc, User, EARTH_J2000, PPP};

use ratatui::{
    backend::{Backend, CrosstermBackend},
    crossterm::{
        event::{
            self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind,
        },
        terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
        execute,
    },
    init as ratatui_init,
    Terminal,
};

#[tokio::main]
async fn main() {
    ratatui_init();

    let mut builder = Builder::from_default_env();

    builder
        .target(Target::Stdout)
        .format_timestamp_secs()
        .format_module_path(false)
        .init();


    let rt_navi = RtNavi::new();

    let mut stdout = io::stdout();
        
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)
        .unwrap_or_else(|e| {
            panic!("Application execution failure: {}", e);
        });

    let backend = CrosstermBackend::new(stdout);

    let mut terminal = Terminal::new(backend)
        .unwrap_or_else(|e| {
            panic!("Application execution failure: {}", e);
        });

    run_app(&mut terminal, rt_navi)
        .unwrap_or_else(|_| {
            panic!("Application execution failure!");
        });

    // // cli and user args
    // let cli = Cli::new();

    // let almanac = Almanac::until_2035().unwrap_or_else(|e| {
    //     panic!("Failed to obtain Almanac definition: {}", e);
    // });

    // let frame = almanac.frame_from_uid(EARTH_J2000).unwrap_or_else(|e| {
    //     panic!("Failed to obtain Frame definition: {}", e);
    // });

    // let sampling_period_nanos = 10_000_000_000;

    // // create channels
    // let (ublox_tx, mut ublox_rx) = mpsc::channel(16);

    // let bias = BiasModels {};
    // let time_source = Time {};

    // let kepler_buf = Rc::new(KeplerBuffer::new());

    // let mut user_profile = User::default();
    // user_profile.profile = None;

    // let mut cfg = Config::static_preset(Method::SPP);

    // cfg.min_sv_elev = Some(10.0);
    // cfg.solver.max_gdop = 10.0;

    // debug!("deployed with {} {:#?}", user_profile, cfg);

    // let mut ppp = PPP::new_survey(
    //     almanac,
    //     frame,
    //     cfg,
    //     Rc::clone(&kepler_buf),
    //     time_source,
    //     bias,
    // );

    // info!("solver deployed");
    // debug!("deploying ublox..");

    // let serial_port_opts = cli.serial_port_opts();

    // let mut ublox = Ublox::new(serial_port_opts, ublox_tx);

    // ublox.init(sampling_period_nanos);

    // tokio::spawn(async move {
    //     ublox.tasklet().await;
    // });

    // info!("rt-navi deployed");

    // loop {
    //     while let Some(msg) = ublox_rx.recv().await {
    //         match msg {
    //             Message::Proposal(candidates) => {
    //                 let epoch = candidates[0].t;
    //                 debug!("{} - new proposal ({} candidates)", epoch, candidates.len());

    //                 match ppp.resolve(user_profile, epoch, &candidates) {
    //                     Ok(pvt) => {
    //                         let (lat, long, alt) = pvt.lat_long_alt_deg_deg_m;

    //                         info!("new solution | lat={:.6}° long={:.6} alt={:.6}m | dt={:.6E}s drift={:.6E}s/s",
    //                             lat, long, alt, pvt.clock_offset_s, pvt.clock_drift_s_s);
    //                     },
    //                     Err(e) => {
    //                         error!("ppp error: {}", e);
    //                         ppp.reset();
    //                     },
    //                 }
    //             },

    //             Message::Kepler(keplerian) => {
    //                 kepler_buf.latch(keplerian);
    //             },
    //         }
    //     }
    // }
}

fn run_app<B: Backend>(terminal: &mut Terminal<B>, mut rt_navi: RtNavi) -> std::io::Result<()> {
    let tick_rate = Duration::from_millis(10);
    let mut last_tick = Instant::now();

    loop {
        update_app_state(&mut rt_navi);
        terminal.draw(|frame| ui::draw(frame, &mut rt_navi))?;

        let timeout = tick_rate.saturating_sub(last_tick.elapsed());

        if event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                match key.code {
                    KeyCode::Left | KeyCode::Char('h') => rt_navi.on_left_key_press(),
                    KeyCode::Right | KeyCode::Char('l') => rt_navi.on_right_key_press(),
                    KeyCode::Char(c) => rt_navi.on_key_press(c),
                    _ => {},
                }
            }
        }

        if last_tick.elapsed() >= tick_rate {
            last_tick = Instant::now();
        }

        if rt_navi.should_quit {
            return Ok(());
        }
    }
}

fn update_app_state(rt_navi: &mut RtNavi) {}
