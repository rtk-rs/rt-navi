use crate::kepler::SVKepler;

use ublox::{GpsEphFrame1, GpsEphFrame2, GpsEphFrame3, GpsFrame, GpsSubframe};

use gnss_rtk::prelude::{ClockCorrection, Duration, Epoch, TimeScale, SV};

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
                    if (frame1.iodc as u8) == frame2.iode && frame2.iode == frame3.iode {
                        return true;
                    }
                }
            }
        }

        false
    }

    pub fn to_kepler(&self) -> Option<SVKepler> {
        let (frame1, frame2, frame3) = (
            self.frame1.as_ref()?,
            self.frame2.as_ref()?,
            self.frame3.as_ref()?,
        );

        Some(SVKepler::from_gps(self.sv, frame1, frame2, frame3))
    }

    pub fn week_number(t_gpst: Epoch, wn: u16) -> u32 {
        let current_week = t_gpst.to_time_of_week().0;
        let delta = current_week - wn as u32;
        let rollover = (delta as f64 / 1024.0).round() as u32;
        wn as u32 + rollover * 1024
    }

    fn toc_gpst(t_gpst: Epoch, frame1: &GpsEphFrame1) -> Epoch {
        let toc_nanos = (frame1.toc as u64) * 1_000_000_000;
        let week = Self::week_number(t_gpst, frame1.week);
        Epoch::from_time_of_week(week, toc_nanos, TimeScale::GPST)
    }

    pub fn clock_correction(&self, t: Epoch) -> Option<ClockCorrection> {
        let frame1 = self.frame1.as_ref()?;

        let t_gpst = t.to_time_scale(TimeScale::GPST);

        let (a0, a1, a2) = (frame1.af0, frame1.af1, frame1.af2);

        let mut dt = (Self::toc_gpst(t_gpst, frame1) - t_gpst).to_seconds();

        for _ in 0..10 {
            dt -= a0 + a1 * dt + a2 * dt.powi(2);
        }

        let dt = Duration::from_seconds(a0 + a1 * dt + a2 * dt.powi(2));
        Some(ClockCorrection::without_relativistic_correction(dt))
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

    pub fn clock_correction(&self, t: Epoch, sv: SV) -> Option<ClockCorrection> {
        if let Some(buff) = self.buffer.iter().find(|buf| buf.sv == sv) {
            buff.clock_correction(t)
        } else {
            None
        }
    }
}
