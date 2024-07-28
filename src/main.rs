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
use ublox::Ublox;

#[derive(Debug, Error)]
pub enum Error {}

pub fn main() -> Result<(), Error> {
    let mut builder = Builder::from_default_env();
    builder
        .target(Target::Stdout)
        .format_timestamp_secs()
        .format_module_path(false)
        .init();

    // cli and user args
    let cli = Cli::new();
    let opts = cli.serial_opts();
    let mut ublox = Ublox::new(opts);

    let method = Method::SPP;
    let cfg = Config::static_preset(method);

    let solver = Solver::new(&cfg, None, |t, sv, _| None);

    Ok(())
}
