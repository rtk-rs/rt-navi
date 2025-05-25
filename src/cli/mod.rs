use std::{
    path::{Path, PathBuf},
    str::FromStr,
};

mod ubx;
pub use ubx::CommandBuilder as UbxCommandBuilder;

use clap::{value_parser, Arg, ArgAction, ArgMatches, ColorChoice, Command};

pub struct SerialPortOpts {
    pub port_name: String,
    pub port_baud: u32,
}

pub struct Cli {
    /// Arguments passed by user
    pub matches: ArgMatches,
}

impl Cli {
    /// Build new command line interface
    pub fn new() -> Self {
        let ubx_cmd = UbxCommandBuilder::new();

        let cmd = 
            Command::new("rt-navi")
                .author("Guillaume W. Bres, <guillaume.bressaix@gmail.com>")
                .version(env!("CARGO_PKG_VERSION"))
                .about("High precision Navigation, in real time")
                .arg_required_else_help(true)
                .color(ColorChoice::Always)
                .next_help_heading("GNSS Receiver (Hardware)")
                .subcommand(ubx_cmd)
                .arg(
                    Arg::new("port")
                        .short('p')
                        .long("port")
                        .required(true)
                        .value_parser(value_parser!(String))
                        .long_help("Define serial port (ex: /dev/ttyACM0 on Linux)"),
                )
                .arg(
                    Arg::new("baud")
                        .long("baud")
                        .short('b')
                        .required(false)
                        .value_parser(value_parser!(u32))
                        .default_value("9600")
                        .long_help("Serial port baud rate"),
                );

            
        #[cfg(feature = "rtcm")]
        let cmd = cmd.next_help_heading("RTCM Clients");

        #[cfg(feature = "rtcm")]
        let cmd = cmd.arg(
                Arg::new("rtcm")
                    .long("rtcm")
                    .required(false)
                    .value_name("URL")
                    .action(ArgAction::Append)
                    .help("Connect to given RTCM host.
Add as many as you need. You should use RTCM links which short baselines (local area).
A minimum of one RTCM link is required for RTK navigation technique."));

        Self {
            matches: cmd.get_matches(),
        }
    }

    pub fn serial_port_opts(&self) -> SerialPortOpts {
        SerialPortOpts {
            port_name: self.matches.get_one::<String>("port").unwrap().to_string(),
            port_baud: *self.matches.get_one::<u32>("baud").unwrap(),
        }
    }
}
