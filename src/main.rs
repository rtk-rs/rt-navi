//! High precision navigation, in real time

// private
mod cli;
mod ublox;

use env_logger::{Builder, Target};

use gnss_rtk::prelude::{Config, Method, Solver};

#[macro_use]
extern crate log;

use cli::Cli;
use thiserror::Error;

use ublox::{Command, Message, Ublox};

use tokio::sync::mpsc;

#[derive(Debug, Error)]
pub enum Error {}

fn main() -> Result<(), Error> {
    let mut builder = Builder::from_default_env();
    builder
        .target(Target::Stdout)
        .format_timestamp_secs()
        .format_module_path(false)
        .init();

    // cli and user args
    let cli = Cli::new();
    let opts = cli.serial_opts();

    // create channels
    let (ublox_tx, mut rx) = mpsc::channel(16);
    let (tx, mut ublox_rx) = mpsc::channel(16);

    let method = Method::SPP;
    let cfg = Config::static_preset(method);

    let solver = Solver::new(&cfg, None, |t, sv, _| None);

    // deploy hardware
    let mut ublox = Ublox::new(opts, ublox_rx, ublox_tx);
    ublox.init();
    tokio::spawn(async move {
        ublox.tasklet();
    });

    loop {
        while let Some(msg) = rx.blocking_recv() {
            match msg {
                Message::Candidates(candidates) => {},
            }
        }
    }
}
