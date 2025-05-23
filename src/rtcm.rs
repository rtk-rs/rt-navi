//! RTCM client
use std::io::Read;

use tokio::{io::AsyncReadExt, net::TcpStream, sync::mpsc::Sender};

use rtcm_rs::{next_msg_frame, MessageBuilder, MessageFrame, MsgFrameIter};

use crate::Message;

/// [RtcmClient] allows to connect to one ntrip caster.
#[derive(Clone)]
pub struct RtcmClient {
    caster: String,
    port: u16,
    tx: Sender<Message>,
}

impl RtcmClient {
    pub fn new(host: String, port: u16, tx: Sender<Message>) -> Self {
        Self {
            caster: host,
            port,
            tx,
        }
    }

    // pub fn with_username(&self) -> Self {
    // }

    // pub fn with_password(&self)

    pub async fn connect(&mut self) -> std::io::Result<()> {
        let mut ptr = 0;
        let mut buffer = [0u8; 1024];
        let mut stream = TcpStream::connect((self.caster.as_str(), self.port)).await?;

        loop {
            let size = stream.read(&mut buffer).await?;

            if size == 0 {
                info!(
                    "connection to {} ntrip caster has been closed",
                    &self.caster
                );
                return Ok(());
            }

            while ptr < size {
                let (size, msg_frame) = next_msg_frame(&buffer[ptr..]);

                if let Some(frame) = msg_frame {
                    let msg = frame.get_message();
                    match msg {
                        rtcm_rs::Message::Corrupt | rtcm_rs::Message::Empty => {},
                        msg => match self.tx.send(Message::RtcmMessage(msg)).await {
                            Ok(_) => {},
                            Err(e) => {
                                error!("Failed to notify RTCM message: {}", e);
                            },
                        },
                    }
                }

                ptr += size;
            }
        }
    }
}
