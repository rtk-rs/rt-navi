use crate::{cli::SerialPortOpts, ephemeris::EphemerisBuffer, Error};
use chrono::prelude::*;
use std::time::Duration as StdDuration;

mod device;
use device::Device;

use ublox::{
    cfg_val::CfgVal, AlignmentToReferenceTime, CfgLayerSet, CfgMsgAllPortsBuilder, CfgRate,
    CfgRateBuilder, CfgValSetBuilder, GnssFixType, MonVer, NavPvt, PacketRef as UbxPacketRef,
    Position as UbxPosition, RxmRawx, RxmSfrbx, RxmSfrbxInterpreted, UbxPacketRequest,
    Velocity as UbxVelocity,
};

use serialport::{
    DataBits as SerialDataBits, FlowControl as SerialFlowControl, Parity as SerialParity,
    StopBits as SerialStopBits,
};

use tokio::sync::mpsc::Sender;

use gnss_rtk::prelude::{
    Candidate, Carrier, Constellation, Duration, Epoch, Observation, TimeScale, SV,
};

use crate::kepler::SVKepler;

pub enum Message {
    /// Kepler update
    Kepler(SVKepler),

    /// Proposal
    Proposal(Vec<Candidate>),
}

pub struct Ublox {
    /// TX port
    tx: Sender<Message>,

    /// Decoding [Device]
    device: Device,
}

fn gnss_rtk_id(gnss_id: u8) -> Option<Constellation> {
    match gnss_id {
        0 => Some(Constellation::GPS),
        1 => Some(Constellation::SBAS),
        2 => Some(Constellation::Galileo),
        3 => Some(Constellation::BeiDou),
        5 => Some(Constellation::QZSS),
        6 => Some(Constellation::Glonass),
        id => None,
    }
}

impl Ublox {
    /// Builds new [Ublox] device
    pub fn new(opts: SerialPortOpts, tx: Sender<Message>) -> Self {
        let port = serialport::new(&opts.port_name, opts.port_baud)
            .stop_bits(SerialStopBits::One)
            .data_bits(SerialDataBits::Eight)
            .timeout(StdDuration::from_millis(10))
            .parity(SerialParity::Even)
            .flow_control(SerialFlowControl::None)
            .open()
            .unwrap_or_else(|e| {
                panic!("Failed to open port {}: {}", opts.port_name, e);
            });

        Self {
            tx,
            device: Device::new(port),
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
        cfg_data.push(CfgVal::UndocumentedL5Enable(true));

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

        debug!("UBX-RXM-RAWX");

        self.device
            .write_all(
                &CfgMsgAllPortsBuilder::set_rate_for::<RxmRawx>([1, 1, 1, 1, 1, 1])
                    .into_packet_bytes(),
            )
            .unwrap_or_else(|e| panic!("Failed to activate RxmRawx msg: {}", e));

        debug!("UBX-NAV-PVT");

        self.device
            .write_all(
                &CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>([1, 1, 1, 1, 1, 1])
                    .into_packet_bytes(),
            )
            .unwrap_or_else(|e| panic!("Failed to activate NavPvt msg: {}", e));

        debug!("UBX-RXM-SFRBX");

        self.device
            .write_all(
                &CfgMsgAllPortsBuilder::set_rate_for::<RxmSfrbx>([1, 1, 1, 1, 1, 1])
                    .into_packet_bytes(),
            )
            .unwrap_or_else(|e| panic!("Failed to activate RxmSfrbx msg: {}", e));

        debug!("UBX-RXM-RATE");

        let measure_rate_ms = (sampling_period_nanos / 1_000_000) as u16;
        let time_ref = AlignmentToReferenceTime::Gps;

        self.device
            .write_all(
                &CfgRateBuilder {
                    measure_rate_ms,
                    nav_rate: 10,
                    time_ref,
                }
                .into_packet_bytes(),
            )
            .unwrap_or_else(|e| panic!("Failed to set measurement rate: {}", e));

        self.device.wait_for_ack::<CfgRate>().unwrap_or_else(|e| {
            panic!("Failed to set measurement rate: {}", e);
        });

        debug!("UBX-MON-VER");

        self.device
            .write_all(&UbxPacketRequest::request_for::<MonVer>().into_packet_bytes())
            .unwrap_or_else(|e| panic!("Failed to request UBX-MON-VER: {}", e));

        let _ = self.device.on_data_available(|packet| match packet {
            UbxPacketRef::MonVer(monver) => {
                debug!(
                    "ublox: hw-version: {}, sw-version: {} , extensions: {:?}",
                    monver.hardware_version(),
                    monver.software_version(),
                    monver.extension().collect::<Vec<&str>>(),
                );
            },
            _ => {},
        });

        debug!("ublox initialized!");
    }

    /// Main tasklet
    pub async fn tasklet(&mut self) {
        let mut latest_t = Epoch::default();
        let mut eph_buffer = EphemerisBuffer::new();
        let mut candidates = Vec::<Candidate>::with_capacity(8);

        loop {
            match self.device.on_data_available(|packet| match packet {
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

                    latest_t = t_gpst;

                    candidates.clear();
                    debug!("{} new measurements", t_gpst);

                    for meas in rawx.measurements() {
                        let gnss_id = meas.gnss_id();

                        let carrier = Carrier::L1;

                        let constell = match gnss_rtk_id(gnss_id) {
                            Some(constell) => constell,
                            None => {
                                error!("non supported constellation: {}", gnss_id);
                                continue;
                            },
                        };

                        let sv = SV::new(constell, meas.sv_id());

                        if let Some(candidate) = candidates.iter_mut().find(|cd| cd.sv == sv) {
                            candidate.set_pseudo_range_m(carrier, meas.pr_mes());

                            // In cycles !
                            // candidate.set_ambiguous_phase_range_m(carrier, meas.cp_mes());

                            if let Some(corr) = eph_buffer.clock_correction(t_gpst, sv) {
                                candidate.set_clock_correction(corr);
                            }

                            if let Some(tgd) = eph_buffer.tgd(sv) {
                                candidate.set_group_delay(tgd);
                            }
                        } else {
                            let observation =
                                Observation::pseudo_range(carrier, meas.pr_mes(), None);

                            // In cycles !
                            //         .with_ambiguous_phase_range_m(meas.cp_mes());

                            let mut cd = Candidate::new(sv, t_gpst, vec![observation]);

                            if let Some(corr) = eph_buffer.clock_correction(t_gpst, sv) {
                                cd.set_clock_correction(corr);
                            }

                            if let Some(tgd) = eph_buffer.tgd(sv) {
                                cd.set_group_delay(tgd);
                            }

                            candidates.push(cd);
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

                        info!(
                            "ublox lat={:.6}° long={:.6}° alt={:.3}m",
                            pos.lat, pos.lon, pos.alt
                        );

                        info!("ublox vel={:.2}m/s heading={:.2}°", vel.speed, vel.heading);
                    }

                    if has_time {
                        let time: DateTime<Utc> = (&sol)
                            .try_into()
                            .expect("Could not parse NAV-PVT time field to UTC");

                        info!("ublox absolute time: {}", time);
                    }
                },

                UbxPacketRef::RxmSfrbx(sfrbx) => {
                    match gnss_rtk_id(sfrbx.gnss_id()) {
                        Some(constellation) => {
                            let sv = SV::new(constellation, sfrbx.sv_id());
                            match constellation {
                                Constellation::GPS | Constellation::QZSS => {
                                    // // DEBUG
                                    // for (index, dword) in sfrbx.dwrd().enumerate() {
                                    //     debug!(
                                    //         "UBX-SFRBX ({}) - dword #{} value=0x{:08x}",
                                    //         sv, index, dword,
                                    //     );
                                    // }
                                    // debug!("\n");
                                    if let Some(interprated) = sfrbx.interprete() {
                                        match interprated {
                                            RxmSfrbxInterpreted::GpsQzss(gps) => {
                                                debug!("UBX-SFRBX ({}) - {:?}", sv, gps);
                                                eph_buffer.update(sv, gps);
                                            },
                                        }
                                    }
                                },
                                Constellation::Glonass => {
                                    let _freq_id = sfrbx.freq_id();
                                },
                                _ => {},
                            }
                        },
                        None => {
                            // error!("Non supported constellation: #{}", sfrbx.gnss_id());
                        },
                    }
                },

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

            for eph in eph_buffer.buffer.iter() {
                if let Some(validated) = eph.validate() {
                    let keplerian = validated.to_kepler(latest_t);

                    match self.tx.send(Message::Kepler(keplerian)).await {
                        Ok(_) => {},
                        Err(e) => {
                            error!("Failed to update {} keplerian state: {}", eph.sv, e);
                        },
                    }
                }
            }
        }
    }
}
