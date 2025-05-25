use std::{
    path::{Path, PathBuf},
    str::FromStr,
};

mod ubx;
pub use ubx::CommandBuilder as UbxCommandBuilder;

use clap::{value_parser, Arg, ArgAction, ArgMatches, ColorChoice, Command};

#[cfg(feature = "rtcm")]
use crate::ntrip::NTRIPInfos;

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
        let cmd = cmd.next_help_heading("NTRIP Client");

        #[cfg(feature = "rtcm")]
        let cmd = cmd.arg(
                Arg::new("ntrip")
                    .long("ntrip")
                    .required(false)
                    .value_name("host:port/mountpoint/user=$name,password=$password")
                    .value_parser(value_parser!(NTRIPInfos))
                    .action(ArgAction::Append)
                    .help("Connect to NTRIP server.
Supports NTRIP V1 and V2, and two formats:
 (1) Passwordless connection: --ntrip host:port/mount
 (2) With credentials: --ntrip host:port/mount/user=$name,password=$pwd

Example: --ntrip 192.168.1.10:1234/custom
Example: --ntrip caster.centipede.fr:2101/PARIS/user=centipede,password=centipede."));

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
