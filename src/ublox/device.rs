use std::{
    io::{ErrorKind as IoErrorKind, Result as IoResult},
    time::{Duration, SystemTime},
};

use serialport::SerialPort;

use ublox::{AnyPacketRef, PacketRef, Parser, UbxPacketMeta};

use rtcm_rs::{next_msg_frame, Message as RtcmMessage, MessageFrame};

pub trait UbxPacketHandler {
    fn handle(&mut self, _packet: PacketRef<'_>) {}
}

/// Implement handler for simple callbacks / closures
impl<F: FnMut(PacketRef)> UbxPacketHandler for F {
    fn handle(&mut self, package: PacketRef) {
        self(package)
    }
}

pub struct Device {
    /// UBX [Parser]
    parser: Parser<Vec<u8>>,

    /// [SerialPort]
    port: Box<dyn SerialPort>,
}

impl Device {
    /// Creates a new U-Blox [Device] parser
    pub fn new(port: Box<dyn SerialPort>) -> Device {
        let parser = Parser::default();
        Device { port, parser }
    }

    // /// Configure u-Blox port
    // pub fn configure_port(
    //     &mut self,
    //     port_config: Option<UbxPortConfiguration>,
    // ) {
    //     if let Some(config) = port_config {

    //         println!("Configuring '{}' port ...", config.port_name.to_uppercase());

    //         self.write_all(
    //             &CfgPrtUartBuilder {
    //                 portid: config.port_id.unwrap(),
    //                 reserved0: 0,
    //                 tx_ready: 0,
    //                 mode: UartMode::new(config.data_bits, config.parity, config.stop_bits),
    //                 baud_rate: config.baud_rate,
    //                 in_proto_mask: config.in_proto_mask,
    //                 out_proto_mask: config.out_proto_mask,
    //                 flags: 0,
    //                 reserved5: 0,
    //             }
    //             .into_packet_bytes(),
    //         )
    //         .unwrap_or_else(|e| {
    //             panic!("U-Blox port configuration failed with: {}", e);
    //         });

    //         self.wait_for_ack::<CfgPrtUart>()
    //             .unwrap_or_else(|e| {
    //                 panic!("U-Blox port configuration failed with: {}", e);
    //             });
    //     }
    // }

    /// Writes all data to serial port
    pub fn write_all(&mut self, data: &[u8]) -> IoResult<()> {
        self.port.write_all(data)
    }

    pub fn on_data_available<F: FnMut(ublox::PacketRef)>(
        &mut self,
        mut callback: F,
    ) -> IoResult<()> {
        self.process(&mut callback)
    }

    ///
    pub fn process(&mut self, handler: &mut impl UbxPacketHandler) -> std::io::Result<()> {
        loop {
            const MAX_PAYLOAD_LEN: usize = 1240;
            let mut local_buf = [0; MAX_PAYLOAD_LEN];
            let nbytes = self.read_port(&mut local_buf)?;
            if nbytes == 0 {
                break;
            }

            // parser.consume_ubx adds the buffer to its internal buffer, and
            // returns an iterator-like object we can use to process the packets
            let mut it = self.parser.consume_ubx_rtcm(&local_buf[..nbytes]);
            loop {
                match it.next() {
                    Some(Ok(packet)) => match packet {
                        AnyPacketRef::Ubx(packet) => handler.handle(packet),
                        #[cfg(not(feature = "rtcm"))]
                        AnyPacketRef::Rtcm(_) => {
                            // RTCM not handled
                        },
                        #[cfg(feature = "rtcm")]
                        AnyPacketRef::Rtcm(packet) => {
                            let (_, frame) = next_msg_frame(packet.data);
                            if let Some(frame) = frame {
                                match frame.get_message() {
                                    RtcmMessage::Msg1001(msg) => {
                                        trace!("RTCM 1001: {:?}", msg);
                                    },
                                    _ => {},
                                }
                            }
                        },
                    },
                    Some(Err(e)) => {
                        eprintln!("Malformed packet, ignore it; cause {e}");
                    },
                    None => {
                        // debug!("Parsed all data in buffer ...");
                        break;
                    },
                }
            }
        }
        Ok(())
    }

    /// Wait for message Acknowledge
    pub fn wait_for_ack<T: UbxPacketMeta>(&mut self) -> IoResult<()> {
        let mut found_packet = false;
        let start = SystemTime::now();
        let timeout = Duration::from_secs(3);

        while !found_packet {
            self.on_data_available(|packet| {
                if let PacketRef::AckAck(ack) = packet {
                    if ack.class() == T::CLASS && ack.msg_id() == T::ID {
                        found_packet = true;
                    }
                }
            })?;

            if start.elapsed().unwrap().as_millis() > timeout.as_millis() {
                eprintln!("Did not receive ACK message for request");
                break;
            }
        }

        Ok(())
    }

    /// Reads the serial port, converting timeouts into "no data received"
    fn read_port(&mut self, output: &mut [u8]) -> std::io::Result<usize> {
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
}
