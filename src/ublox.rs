use crate::Error;
use chrono::prelude::*;
use std::time::Duration as StdDuration;

use ublox::{
    cfg_val::CfgVal, AlignmentToReferenceTime, CfgLayerSet, CfgMsgAllPorts, CfgMsgAllPortsBuilder,
    CfgRate, CfgRateBuilder, CfgValSetBuilder, GnssFixType, NavEoe, NavPvt,
    PacketRef as UbxPacketRef, Parser as UbxParser, Position as UbxPosition, RxmRawx,
    UbxPacketMeta, Velocity as UbxVelocity,
};

use std::io::{ErrorKind as IoErrorKind, Result as IoResult};

use serialport::{
    DataBits as SerialDataBits, FlowControl as SerialFlowControl, Parity as SerialParity,
    SerialPort, StopBits as SerialStopBits,
};

use tokio::sync::mpsc::Sender;

use gnss_rtk::prelude::{Candidate, Carrier, Constellation, Epoch, Observation, TimeScale, SV};

#[derive(Debug, Clone)]
pub enum Message {
    Proposal(Vec<Candidate>),
}

pub struct SerialOpts {
    pub port: String,
    pub baud: u32,
}

pub struct Ublox {
    /// TX port
    tx: Sender<Message>,

    /// Serial port
    port: Box<dyn SerialPort>,

    /// Parser
    parser: UbxParser<Vec<u8>>,
}

fn gnss_rtk_id(gnss_id: u8) -> Result<Constellation, Error> {
    match gnss_id {
        0 => Ok(Constellation::GPS),
        1 => Ok(Constellation::SBAS),
        2 => Ok(Constellation::Galileo),
        3 => Ok(Constellation::BeiDou),
        5 => Ok(Constellation::QZSS),
        6 => Ok(Constellation::Glonass),
        id => Err(Error::NonSupportedGnss(id)),
    }
}

impl Ublox {
    /// Builds new [Ublox] device
    pub fn new(opts: SerialOpts, tx: Sender<Message>) -> Self {
        let port = opts.port.clone();
        let port = serialport::new(opts.port, opts.baud)
            .stop_bits(SerialStopBits::One)
            .data_bits(SerialDataBits::Eight)
            .timeout(StdDuration::from_millis(10))
            .parity(SerialParity::Even)
            .flow_control(SerialFlowControl::None)
            .open()
            .unwrap_or_else(|e| {
                panic!("Failed to open port {}: {}", port, e);
            });

        Self {
            tx,
            port,
            parser: Default::default(),
        }
    }

    /// Initialize hardware device
    pub fn init(&mut self, sampling_period_nanos: u64) {
        // activate GPS + QZSS + Galileo
        let mut cfg_data = Vec::<CfgVal>::new();

        cfg_data.push(CfgVal::SignalGpsEna(true));
        cfg_data.push(CfgVal::SignalQzssEna(true));
        cfg_data.push(CfgVal::SignalGpsL1caEna(true));
        cfg_data.push(CfgVal::SignalGpsL2cEna(true));
        cfg_data.push(CfgVal::UndocumentedL5Enable(false));

        cfg_data.push(CfgVal::SignalGloEna(false));
        cfg_data.push(CfgVal::SignalGloL1Ena(false));
        cfg_data.push(CfgVal::SignalGLoL2Ena(false));

        cfg_data.push(CfgVal::SignalGalEna(true));
        cfg_data.push(CfgVal::SignalGalE1Ena(true));
        cfg_data.push(CfgVal::SignalGalE5bEna(true));

        CfgValSetBuilder {
            version: 0,
            layers: CfgLayerSet::RAM,
            reserved1: 0,
            cfg_data: &cfg_data,
        };

        self.write_acked(
            CfgMsgAllPorts,
            &CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>([0, 1, 1, 1, 0, 0]).into_packet_bytes(),
        )
        .unwrap_or_else(|e| panic!("Failed to activate NavPvt msg: {}", e));

        self.write_acked(
            CfgMsgAllPorts,
            &CfgMsgAllPortsBuilder::set_rate_for::<NavEoe>([0, 1, 1, 1, 0, 0]).into_packet_bytes(),
        )
        .unwrap_or_else(|e| panic!("Failed to activate NavEoe msg: {}", e));

        self.write_acked(
            CfgMsgAllPorts,
            &CfgMsgAllPortsBuilder::set_rate_for::<RxmRawx>([0, 1, 1, 1, 0, 0]).into_packet_bytes(),
        )
        .unwrap_or_else(|e| panic!("Failed to activate RxmRawx msg: {}", e));

        let measure_rate_ms = (sampling_period_nanos / 1_000_000) as u16;
        let time_ref = AlignmentToReferenceTime::Gps;

        self.write_all(
            &CfgRateBuilder {
                measure_rate_ms,
                nav_rate: 10,
                time_ref,
            }
            .into_packet_bytes(),
        )
        .unwrap_or_else(|e| panic!("Failed to set measurement rate: {}", e));

        self.wait_for_ack::<CfgRate>().unwrap_or_else(|e| {
            panic!("Failed to set measurement rate: {}", e);
        });
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
            let mut it = self.parser.consume_ubx(&local_buf[..nbytes]);

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
    pub async fn tasklet(&mut self) {
        let mut candidates = Vec::<Candidate>::with_capacity(8);

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
                UbxPacketRef::RxmRawx(rawx) => {
                    let week = rawx.week() as u32;
                    let nanos = rawx.rcv_tow().round() as u64 * 1_000_000_000;
                    let t_gpst = Epoch::from_time_of_week(week, nanos, TimeScale::GPST);

                    debug!("{} new measurements", t_gpst);

                    candidates.clear();

                    for meas in rawx.measurements() {
                        let gnss_id = meas.gnss_id();

                        let carrier = Carrier::L1;

                        let constell = match gnss_rtk_id(gnss_id) {
                            Ok(constell) => constell,
                            Err(e) => {
                                error!("non supported constellation: {}", e);
                                continue;
                            },
                        };

                        let sv = SV::new(constell, meas.sv_id());

                        if let Some(candidate) = candidates
                            .iter_mut()
                            .find(|cd| cd.t == t_gpst && cd.sv == sv)
                        {
                            candidate.set_pseudo_range_m(carrier, meas.pr_mes());
                            candidate.set_ambiguous_phase_range_m(carrier, meas.cp_mes());
                        } else {
                            let observations = vec![
                                Observation::pseudo_range(carrier, meas.pr_mes(), None),
                                Observation::ambiguous_phase_range(carrier, meas.cp_mes(), None),
                            ];

                            candidates.push(Candidate::new(sv, t_gpst, observations));
                        }
                    }
                },
                UbxPacketRef::NavPvt(sol) => {
                    let has_time = sol.fix_type() == GnssFixType::Fix3D
                        || sol.fix_type() == GnssFixType::GPSPlusDeadReckoning
                        || sol.fix_type() == GnssFixType::TimeOnlyFix;

                    let has_posvel = sol.fix_type() == GnssFixType::Fix3D
                        || sol.fix_type() == GnssFixType::GPSPlusDeadReckoning;

                    if has_posvel {
                        let pos: UbxPosition = (&sol).into();
                        let vel: UbxVelocity = (&sol).into();

                        debug!(
                            "ublox lat={:.6}° long={:.6}° alt={:.3}m",
                            pos.lat, pos.lon, pos.alt
                        );

                        debug!("ublox vel={:.2}m/s heading={:.2}°", vel.speed, vel.heading);
                    }

                    if has_time {
                        let time: DateTime<Utc> = (&sol)
                            .try_into()
                            .expect("Could not parse NAV-PVT time field to UTC");

                        debug!("ublox absolute time: {}", time);
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
                    trace!("unhandled message {:?}", packet);
                },
            }) {
                Ok(_) => {},
                Err(e) => {
                    error!("ublox error: {}", e);
                },
            }

            if candidates.len() > 0 {
                match self.tx.send(Message::Proposal(candidates.clone())).await {
                    Ok(_) => {},
                    Err(e) => {
                        error!("Failed to propose new candidates: {}", e);
                    },
                }

                candidates.clear();
            }
        }
    }
}
