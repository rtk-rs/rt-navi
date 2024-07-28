//! High precision navigation, in real time

// private
mod cli;

use env_logger::{Builder, Target};

#[macro_use]
extern crate log;

use cli::Cli;
use thiserror::Error;

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

    Ok(())
}
