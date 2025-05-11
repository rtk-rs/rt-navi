use clap::{value_parser, Arg, Command};
use log::debug;
use serialport::{FlowControl as SerialFlowControl, SerialPort};
use std::time::Duration;

pub struct CommandBuilder {}

impl CommandBuilder {
    pub fn new() -> Command {
        let command = Command::new("ublox")
                .about("Select configuration settings for specific UART/USB port to send to uBlox as configuration message")
                .arg(
                    Arg::new("port")
                        .long("select")
                        .required(true)
                        .default_value("usb")
                        .value_parser(value_parser!(String))
                        .long_help(
                            "Apply specific configuration to the selected port. Supported: usb, uart1, uart2. Configuration includes: protocol in/out, data-bits, stop-bits, parity, baud-rate",
                        ),
                    )
                .arg(
                    Arg::new("cfg-baud")
                        .value_name("baud")
                        .long("baud")
                        .required(false)
                        .default_value("9600")
                        .value_parser(value_parser!(u32))
                        .help("Baud rate to set"),
                )
                .arg(
                    Arg::new("stop-bits")
                        .long("stop-bits")
                        .help("Number of stop bits to set")
                        .required(false)
                        .value_parser(["1", "2"])
                        .default_value("1"),
                )
                .arg(
                    Arg::new("data-bits")
                        .long("data-bits")
                        .help("Number of data bits to set")
                        .required(false)
                        .value_parser(["7", "8"])
                        .default_value("8"),
                )
                .arg(
                    Arg::new("parity")
                        .long("parity")
                        .help("Parity to set")
                        .required(false)
                        .value_parser(["even", "odd"]),
                )
                .arg(
                    Arg::new("in-ublox")
                        .long("in-ublox")
                        .default_value("true")
                        .action(clap::ArgAction::SetTrue)
                        .help("Toggle receiving UBX proprietary protocol on port"),
                )
                .arg(
                    Arg::new("in-nmea")
                        .long("in-nmea")
                        .default_value("false")
                        .action(clap::ArgAction::SetTrue)
                        .help("Toggle receiving NMEA protocol on port"),
                )
                .arg(
                    Arg::new("in-rtcm")
                        .long("in-rtcm")
                        .default_value("false")
                        .action(clap::ArgAction::SetTrue)
                        .help("Toggle receiving RTCM protocol on port"),
                )
                .arg(
                    Arg::new("in-rtcm3")
                        .long("in-rtcm3")
                        .default_value("false")
                        .action(clap::ArgAction::SetTrue)
                        .help(
                            "Toggle receiving RTCM3 protocol on port. Not supported on uBlox protocol versions below 20",
                        ),
                )
                .arg(
                    Arg::new("out-ublox")
                        .long("out-ublox")
                        .action(clap::ArgAction::SetTrue)
                        .help("Toggle sending UBX proprietary protocol on port"),
                )
                .arg(
                    Arg::new("out-nmea")
                        .long("out-nmea")
                        .action(clap::ArgAction::SetTrue)
                        .help("Toggle sending NMEA protocol on port"),
                )
                .arg(
                    Arg::new("out-rtcm3")
                        .long("out-rtcm3")
                        .action(clap::ArgAction::SetTrue)
                        .help(
                            "Toggle sending RTCM3 protocol on port. Not supported on uBlox protocol versions below 20",
                        ),
                );

        command
    }
}
