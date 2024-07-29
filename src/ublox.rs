use crate::Error;
use chrono::prelude::*;
use std::time::Duration as StdDuration;

use ublox::{
    CfgMsgAllPorts, CfgMsgAllPortsBuilder, GpsFix, MgaGloEph, MgaGpsEph, MgaGpsIono, NavEoe,
    NavHpPosEcef, NavPvt, PacketRef as UbxPacketRef, Parser as UbxParser, Position as UbxPosition,
    RxmRawx, UbxPacketMeta, Velocity as UbxVelocity,
};

use std::io::{ErrorKind as IoErrorKind, Result as IoResult};

use serialport::{
    DataBits as SerialDataBits, FlowControl as SerialFlowControl, Parity as SerialParity,
    SerialPort, StopBits as SerialStopBits,
};

use tokio::sync::mpsc::{Receiver, Sender};

use gnss_rtk::prelude::{
    Candidate, Carrier, Constellation, Duration, Epoch, PhaseRange, PseudoRange, SV,
};

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

#[derive(Debug, Clone, Copy, Default)]
struct Tow {
    tow: u32,
    week: u32,
}

impl Tow {
    fn epoch(&self) -> Epoch {
        Default::default()
    }
}

pub struct Ublox {
    rx: Receiver<Command>,
    tx: Sender<Message>,
    port: Box<dyn SerialPort>,
    parser: UbxParser<Vec<u8>>,
}

fn gnss_rtk_id(gnss_id: u8) -> Result<Constellation, Error> {
    match gnss_id {
        0 => Ok(Constellation::GPS),
        1 => Ok(Constellation::Galileo),
        id => Err(Error::NonSupportedGnss(id)),
    }
}

fn freq_rtk_id(freq_id: u8) -> Result<Carrier, Error> {
    match freq_id {
        0 => Ok(Carrier::default()),
        id => Err(Error::NonSupportedSignal(id)),
    }
}

impl Ublox {
    /// Builds new Ublox device
    pub fn new(opts: SerialOpts, rx: Receiver<Command>, tx: Sender<Message>) -> Self {
        let port = opts.port.clone();
        let port = serialport::new(opts.port, opts.baud)
            .stop_bits(SerialStopBits::One)
            .data_bits(SerialDataBits::Eight)
            .timeout(StdDuration::from_millis(10))
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

        self.write_acked(
            CfgMsgAllPorts,
            &CfgMsgAllPortsBuilder::set_rate_for::<NavEoe>([0, 1, 1, 1, 0, 0]).into_packet_bytes(),
        )
        .unwrap_or_else(|e| panic!("failed to activate NavEoe msg: {}", e));

        self.write_acked(
            CfgMsgAllPorts,
            &CfgMsgAllPortsBuilder::set_rate_for::<RxmRawx>([0, 1, 1, 1, 0, 0]).into_packet_bytes(),
        )
        .unwrap_or_else(|e| panic!("failed to activate RxmRawx msg: {}", e));
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
        let mut sv = SV::default();
        let mut tow = Tow::default();
        let mut carrier = Carrier::default();
        let mut gnss = Constellation::default();
        let mut candidates = Vec::<Candidate>::with_capacity(16);
        loop {
            match self.update(|packet| match packet {
                UbxPacketRef::MonVer(packet) => {
                    info!(
                        "SW version: {} HW version: {}; Extensions: {:?}",
                        packet.software_version(),
                        packet.hardware_version(),
                        packet.extension().collect::<Vec<&str>>()
                    );
                },
                UbxPacketRef::NavEoe(_) => {},
                UbxPacketRef::RxmRawx(rawx) => {
                    debug!("{} new measurements", rawx.num_meas());
                    for meas in rawx.measurements() {
                        let cno = meas.cno();
                        let freq_id = meas.freq_id();
                        let gnss_id = meas.gnss_id();

                        if let Ok(c) = freq_rtk_id(freq_id) {
                            carrier = c;
                        } else {
                            error!("non supported signal: {}", freq_id);
                        }

                        if let Ok(g) = gnss_rtk_id(gnss_id) {
                            gnss = g;
                        } else {
                            error!("non supported gnss: {}", gnss_id);
                        }

                        let cp_mes = meas.cp_mes();
                        let do_mes = meas.do_mes();
                        let pr_mes = meas.pr_mes();

                        candidates.push(Candidate::new(
                            sv,
                            tow.epoch(),
                            Duration::default(),
                            None,
                            vec![PseudoRange {
                                carrier,
                                value: pr_mes,
                                snr: None, //TODO
                            }],
                            vec![PhaseRange {
                                carrier,
                                value: cp_mes,
                                snr: None,       //TODO
                                ambiguity: None, //TODO ?
                            }],
                        ));
                    }
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
                        info!(
                            "Ubx Latitude: {:.5} Longitude: {:.5} Altitude: {:.2}m",
                            pos.lat, pos.lon, pos.alt
                        );
                        info!(
                            "Ubx Velocity: {:.2} m/s Heading: {:.2} degrees",
                            vel.speed, vel.heading
                        );
                    }

                    if has_time {
                        let time: DateTime<Utc> = (&sol)
                            .try_into()
                            .expect("Could not parse NAV-PVT time field to UTC");
                        println!("Time: {:?}", time);
                    }
                },
                // Others
                UbxPacketRef::InfTest(msg) => {
                    trace!("{:?}", msg);
                },
                UbxPacketRef::InfWarning(msg) => {
                    warn!("{:?}", msg);
                },
                UbxPacketRef::InfDebug(msg) => {
                    debug!("{:?}", msg);
                },
                UbxPacketRef::InfNotice(msg) => {
                    info!("{:?}", msg);
                },
                _ => {
                    trace!("{:?}", packet);
                },
            }) {
                Ok(_) => {},
                Err(e) => {},
            }
        }
    }
}
