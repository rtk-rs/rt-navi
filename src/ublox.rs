use std::time::Duration;
use thiserror::Error;

use chrono::prelude::*;

use ublox::{
    CfgMsgAllPorts, CfgMsgAllPortsBuilder, GpsFix, NavPvt, PacketRef as UbxPacketRef,
    Parser as UbxParser, Position as UbxPosition, UbxPacketMeta, Velocity as UbxVelocity,
};

use std::io::{ErrorKind as IoErrorKind, Result as IoResult};

use serialport::{
    DataBits as SerialDataBits, FlowControl as SerialFlowControl, Parity as SerialParity,
    SerialPort, StopBits as SerialStopBits,
};

use tokio::sync::mpsc::{Receiver, Sender};

use gnss_rtk::prelude::Candidate;

#[derive(Debug, Error)]
pub enum Error {}

#[derive(Debug, Clone)]
pub enum Command {
    AbortCandidates,
}

#[derive(Debug, Clone)]
pub enum Message {
    Candidates(Vec<Candidate>),
}

pub struct SerialOpts {
    pub port: String,
    pub baud: u32,
}

pub struct Ublox {
    rx: Receiver<Command>,
    tx: Sender<Message>,
    port: Box<dyn SerialPort>,
    parser: UbxParser<Vec<u8>>,
}

impl Ublox {
    /// Builds new Ublox device
    pub fn new(opts: SerialOpts, rx: Receiver<Command>, tx: Sender<Message>) -> Self {
        let port = opts.port.clone();
        let port = serialport::new(opts.port, opts.baud)
            .stop_bits(SerialStopBits::One)
            .data_bits(SerialDataBits::Eight)
            .timeout(Duration::from_millis(10))
            .parity(SerialParity::Even)
            .flow_control(SerialFlowControl::None)
            .open()
            .unwrap_or_else(|e| {
                panic!("failed to open port {}: {}", port, e);
            });
        Self {
            rx,
            tx,
            port,
            parser: Default::default(),
        }
    }

    /// Initialize hardware device
    pub fn init(&mut self) {
        self.write_acked(
            CfgMsgAllPorts,
            &CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>([0, 1, 1, 1, 0, 0]).into_packet_bytes(),
        )
        .unwrap_or_else(|e| panic!("failed to activate NavPvt msg: {}", e));
    }

    /// Writes all bytes to device
    pub fn write_all(&mut self, data: &[u8]) -> IoResult<()> {
        self.port.write_all(data)
    }

    /// Writes message and waits for ack
    pub fn write_acked<M: UbxPacketMeta>(&mut self, msg: M, data: &[u8]) -> IoResult<()> {
        self.port.write_all(data)?;
        self.wait_for_ack::<M>()
    }

    /// Wait for ACK from device
    pub fn wait_for_ack<T: UbxPacketMeta>(&mut self) -> std::io::Result<()> {
        let mut found_packet = false;
        while !found_packet {
            self.update(|packet| {
                if let UbxPacketRef::AckAck(ack) = packet {
                    if ack.class() == T::CLASS && ack.msg_id() == T::ID {
                        found_packet = true;
                    }
                }
            })?;
        }
        Ok(())
    }

    /// Reads serial port into buffer
    fn read_port(&mut self, output: &mut [u8]) -> IoResult<usize> {
        match self.port.read(output) {
            Ok(b) => Ok(b),
            Err(e) => {
                if e.kind() == IoErrorKind::TimedOut {
                    Ok(0)
                } else {
                    Err(e)
                }
            },
        }
    }

    pub fn update<T: FnMut(UbxPacketRef)>(&mut self, mut cb: T) -> IoResult<()> {
        loop {
            const MAX_PAYLOAD_LEN: usize = 1240;
            let mut local_buf = [0; MAX_PAYLOAD_LEN];
            let nbytes = self.read_port(&mut local_buf)?;
            if nbytes == 0 {
                break;
            }

            // parser.consume adds the buffer to its internal buffer, and
            // returns an iterator-like object we can use to process the packets
            let mut it = self.parser.consume(&local_buf[..nbytes]);
            loop {
                match it.next() {
                    Some(Ok(packet)) => {
                        cb(packet);
                    },
                    Some(Err(_)) => {
                        // Received a malformed packet, ignore it
                    },
                    None => {
                        // We've eaten all the packets we have
                        break;
                    },
                }
            }
        }
        Ok(())
    }

    /// Main tasklet
    pub fn tasklet(&mut self) {
        loop {
            match self.update(|packet| match packet {
                UbxPacketRef::MonVer(packet) => {
                    println!(
                        "SW version: {} HW version: {}; Extensions: {:?}",
                        packet.software_version(),
                        packet.hardware_version(),
                        packet.extension().collect::<Vec<&str>>()
                    );
                    println!("{:?}", packet);
                },
                UbxPacketRef::NavPvt(sol) => {
                    let has_time = sol.fix_type() == GpsFix::Fix3D
                        || sol.fix_type() == GpsFix::GPSPlusDeadReckoning
                        || sol.fix_type() == GpsFix::TimeOnlyFix;
                    let has_posvel = sol.fix_type() == GpsFix::Fix3D
                        || sol.fix_type() == GpsFix::GPSPlusDeadReckoning;

                    if has_posvel {
                        let pos: UbxPosition = (&sol).into();
                        let vel: UbxVelocity = (&sol).into();
                        println!(
                            "Latitude: {:.5} Longitude: {:.5} Altitude: {:.2}m",
                            pos.lat, pos.lon, pos.alt
                        );
                        println!(
                            "Speed: {:.2} m/s Heading: {:.2} degrees",
                            vel.speed, vel.heading
                        );
                        println!("Sol: {:?}", sol);
                    }

                    if has_time {
                        let time: DateTime<Utc> = (&sol)
                            .try_into()
                            .expect("Could not parse NAV-PVT time field to UTC");
                        println!("Time: {:?}", time);
                    }
                },
                _ => {
                    println!("{:?}", packet);
                },
            }) {
                Ok(_) => {},
                Err(e) => {},
            }
        }
    }
}
