use crate::kepler::SVKepler;

use ublox::{GpsEphFrame1, GpsEphFrame2, GpsEphFrame3, GpsFrame, GpsSubframe};

use gnss_rtk::prelude::SV;

#[derive(Debug, Default, Clone)]
pub struct GpsSvRawEphemeris {
    pub sv: SV,
    pub frame1: Option<GpsEphFrame1>,
    pub frame2: Option<GpsEphFrame2>,
    pub frame3: Option<GpsEphFrame3>,
}

impl GpsSvRawEphemeris {
    pub fn is_ready(&self) -> bool {
        if let Some(frame1) = &self.frame1 {
            if let Some(frame2) = &self.frame2 {
                if let Some(frame3) = &self.frame3 {
                    // TODO if (frame1.iodc as u8) == frame2.iode && frame2.iode == frame3.iode {
                    if frame2.iode == frame3.iode {
                        return true;
                    }
                }
            }
        }
        false
    }

    pub fn reset(&mut self) {
        (self.frame1, self.frame2, self.frame3) = (None, None, None);
    }

    pub fn to_kepler(&self) -> Option<SVKepler> {
        let (frame1, frame2, frame3) = (
            self.frame1.as_ref()?,
            self.frame2.as_ref()?,
            self.frame3.as_ref()?,
        );

        Some(SVKepler::from_gps(self.sv, frame1, frame2, frame3))
    }
}

#[derive(Default)]
pub struct EphemerisBuffer {
    pub buffer: Vec<GpsSvRawEphemeris>,
}

impl EphemerisBuffer {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(16),
        }
    }

    pub fn latch_gps_frame(&mut self, sv: SV, frame: GpsFrame) {
        if let Some(gps_sv) = self.buffer.iter_mut().find(|buf| buf.sv == sv) {
            match frame.subframe {
                GpsSubframe::Eph1(eph1) => {
                    gps_sv.frame1 = Some(eph1);
                },
                GpsSubframe::Eph2(eph2) => {
                    gps_sv.frame2 = Some(eph2);
                },
                GpsSubframe::Eph3(eph3) => {
                    gps_sv.frame3 = Some(eph3);
                },
            }
        } else {
            let mut new = GpsSvRawEphemeris::default();
            new.sv = sv;
            self.buffer.push(new);
        }
    }
}
