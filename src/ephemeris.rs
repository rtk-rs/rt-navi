use crate::kepler::SVKepler;

use gnss_protos::{
    GpsQzssFrame, GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3, GpsQzssHow, GpsQzssSubframe,
};

use gnss_rtk::prelude::{ClockCorrection, Duration, Epoch, TimeScale, SV};

#[derive(Debug, Default, Clone)]
pub struct GpsSvEphemeris {
    pub sv: SV,
    pub how: GpsQzssHow,
    pub frame1: GpsQzssFrame1,
    pub frame2: GpsQzssFrame2,
    pub frame3: GpsQzssFrame3,
}

impl GpsSvEphemeris {
    pub fn clock_correction(&self, t: Epoch) -> ClockCorrection {
        let t_gpst = t.to_time_scale(TimeScale::GPST);
        let (a0, a1, a2) = (self.frame1.af0, self.frame1.af1, self.frame1.af2);

        let toc_gpst = self.toc_gpst(t_gpst);
        let mut dt = (t_gpst - toc_gpst).to_seconds();

        for _ in 0..10 {
            dt -= a0 + a1 * dt + a2 * dt.powi(2);
        }

        let dt = Duration::from_seconds(a0 + a1 * dt + a2 * dt.powi(2));
        ClockCorrection::without_relativistic_correction(dt)
    }

    fn week_number(t_gpst: Epoch, wn: u16) -> u32 {
        let current_week = t_gpst.to_time_of_week().0;
        let delta = current_week - wn as u32;
        let rollover = (delta as f64 / 1024.0).round() as u32;
        wn as u32 + rollover * 1024
    }

    pub fn toc_gpst(&self, t_gpst: Epoch) -> Epoch {
        let toc_nanos = (self.frame1.toc as u64) * 1_000_000_000;
        let week = Self::week_number(t_gpst, self.frame1.week);
        Epoch::from_time_of_week(week, toc_nanos, TimeScale::GPST)
    }

    pub fn toe_gpst(&self, t_gpst: Epoch) -> Epoch {
        let toe_nanos = (self.frame2.toe as u64) * 1_000_000_000;
        let week = Self::week_number(t_gpst, self.frame1.week);
        Epoch::from_time_of_week(week, toe_nanos, TimeScale::GPST)
    }

    pub fn to_kepler(&self, t_gpst: Epoch) -> SVKepler {
        let toe_s = self.frame2.toe;
        let toc_gpst = self.toc_gpst(t_gpst);
        let toe_gpst = self.toe_gpst(t_gpst);

        SVKepler::from_gps(
            self.sv,
            toe_s as f64,
            toc_gpst,
            toe_gpst,
            &self.frame1,
            &self.frame2,
            &self.frame3,
        )
    }
}

#[derive(Debug, Default, Clone)]
pub struct GpsPendingSvEphemeris {
    pub sv: SV,
    pub how: GpsQzssHow,
    pub frame1: Option<GpsQzssFrame1>,
    pub frame2: Option<GpsQzssFrame2>,
    pub frame3: Option<GpsQzssFrame3>,
}

impl GpsPendingSvEphemeris {
    pub fn new(sv: SV, frame: GpsQzssFrame) -> Self {
        match frame.subframe {
            GpsQzssSubframe::Ephemeris1(eph1) => Self {
                sv,
                how: frame.how,
                frame2: None,
                frame3: None,
                frame1: Some(eph1),
            },
            GpsQzssSubframe::Ephemeris2(eph2) => Self {
                sv,
                how: frame.how,
                frame3: None,
                frame1: None,
                frame2: Some(eph2),
            },
            GpsQzssSubframe::Ephemeris3(eph3) => Self {
                sv,
                how: frame.how,
                frame2: None,
                frame1: None,
                frame3: Some(eph3),
            },
        }
    }

    pub fn update(&mut self, frame: GpsQzssFrame) {
        self.how = frame.how.clone();

        match frame.subframe {
            GpsQzssSubframe::Ephemeris1(eph1) => {
                self.frame1 = Some(eph1);
            },
            GpsQzssSubframe::Ephemeris2(eph2) => {
                self.frame2 = Some(eph2);
            },
            GpsQzssSubframe::Ephemeris3(eph3) => {
                self.frame3 = Some(eph3);
            },
        }
    }

    pub fn validate(&self) -> Option<GpsSvEphemeris> {
        if let Some(frame1) = self.frame1.as_ref() {
            if let Some(frame2) = self.frame2.as_ref() {
                if let Some(frame3) = self.frame3.as_ref() {
                    if frame2.iode == frame3.iode {
                        if frame1.iodc as u8 == frame2.iode {
                            return Some(GpsSvEphemeris {
                                sv: self.sv,
                                how: self.how.clone(),
                                frame1: frame1.clone(),
                                frame2: frame2.clone(),
                                frame3: frame3.clone(),
                            });
                        }
                    }
                }
            }
        }

        None
    }
}

#[derive(Default)]
pub struct EphemerisBuffer {
    pub buffer: Vec<GpsPendingSvEphemeris>,
}

impl EphemerisBuffer {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(16),
        }
    }

    pub fn update(&mut self, sv: SV, frame: GpsQzssFrame) {
        if let Some(sv_data) = self.buffer.iter_mut().find(|buf| buf.sv == sv) {
            sv_data.update(frame);
        } else {
            let new = GpsPendingSvEphemeris::new(sv, frame);
            self.buffer.push(new);
        }
    }

    pub fn clock_correction(&self, t: Epoch, sv: SV) -> Option<ClockCorrection> {
        self.buffer
            .iter()
            .filter_map(|buf| {
                if buf.sv == sv {
                    if let Some(validated) = buf.validate() {
                        Some(validated.clock_correction(t))
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
    }

    pub fn tgd(&self, sv: SV) -> Option<Duration> {
        self.buffer
            .iter()
            .filter_map(|buf| {
                if buf.sv == sv {
                    if let Some(validated) = buf.validate() {
                        Some(Duration::from_seconds(validated.frame1.tgd))
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .reduce(|k, _| k)
    }
}
