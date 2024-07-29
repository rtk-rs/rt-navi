use std::time::Duration;
use thiserror::Error;

use ublox::{
    CfgMsgAllPorts, CfgMsgAllPortsBuilder, NavPvt, PacketRef as UbxPacketRef, Parser as UbxParser,
    UbxPacketMeta,
};

use std::io::{ErrorKind as IoErrorKind, Result as IoResult};

use serialport::{
    DataBits as SerialDataBits, FlowControl as SerialFlowControl, Parity as SerialParity,
    SerialPort, StopBits as SerialStopBits,
};

use gnss_rtk::prelude::Candidate;

#[derive(Debug, Error)]
pub enum Error {}

pub struct SerialOpts {
    pub port: String,
    pub baud: u32,
}

pub struct Ublox {
    port: Box<dyn SerialPort>,
    parser: UbxParser<Vec<u8>>,
}

#[derive(Debug, Clone)]
pub enum Message {
    Candidates(Vec<Candidate>),
}

impl Ublox {
    /// Builds new Ublox device
    pub fn new(opts: SerialOpts) -> Self {
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
}
