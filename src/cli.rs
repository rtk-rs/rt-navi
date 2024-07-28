use std::{
    path::{Path, PathBuf},
    str::FromStr,
};

use crate::ublox::SerialOpts;
use clap::{value_parser, Arg, ArgAction, ArgMatches, ColorChoice, Command};

pub struct Cli {
    /// Arguments passed by user
    pub matches: ArgMatches,
}

impl Default for Cli {
    fn default() -> Self {
        Self::new()
    }
}

impl Cli {
    /// Build new command line interface
    pub fn new() -> Self {
        Self {
            matches: {
                Command::new("rt-navi")
                    .author("Guillaume W. Bres, <guillaume.bressaix@gmail.com>")
                    .version(env!("CARGO_PKG_VERSION"))
                    .about("High precision Navigation, in real time")
                    .arg_required_else_help(true)
                    .color(ColorChoice::Always)
                    .next_help_heading("GNSS Receiver (Hardware)")
                    .arg(
                        Arg::new("ublox")
                            .short('u')
                            .long("ublox")
                            .required(true)
                            .value_name("PORT")
                            .help("Specify serial port to Ublox device"),
                    )
                    .get_matches()
            },
        }
    }
    pub fn serial_opts(&self) -> SerialOpts {
        SerialOpts {
            port: self.matches.get_one::<String>("ublox").unwrap().to_string(),
            baud: 9600,
        }
    }
}
