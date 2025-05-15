use crate::kepler::SVKepler;

use ublox::{
    RxmSfrbxGpsQzssFrame, RxmSfrbxGpsQzssFrame1, RxmSfrbxGpsQzssFrame2, RxmSfrbxGpsQzssFrame3,
    RxmSfrbxGpsQzssSubframe,
};

use gnss_rtk::prelude::{ClockCorrection, Duration, Epoch, TimeScale, SV};

#[derive(Default, Debug, Copy, Clone)]
pub enum State {
    #[default]
    Collecting,
    Valid,
    Invalid,
}

#[derive(Debug, Default, Clone)]
pub struct GpsSvEphemeris {
    pub sv: SV,
    pub state: State,
    pub frame1: Option<RxmSfrbxGpsQzssFrame1>,
    pub frame2: Option<RxmSfrbxGpsQzssFrame2>,
    pub frame3: Option<RxmSfrbxGpsQzssFrame3>,
}

impl GpsSvEphemeris {
    pub fn update(&mut self, t: Epoch, subframe: RxmSfrbxGpsQzssSubframe) {
        match self.state {
            State::Collecting => {
                match subframe {
                    RxmSfrbxGpsQzssSubframe::Eph1(eph1) => {
                        self.frame1 = Some(eph1);
                    },
                    RxmSfrbxGpsQzssSubframe::Eph2(eph2) => {
                        self.frame2 = Some(eph2);
                    },
                    RxmSfrbxGpsQzssSubframe::Eph3(eph3) => {
                        self.frame3 = Some(eph3);
                    },
                }
                if self.is_complete() {
                    self.state = State::Invalid;
                }
            },
            State::Invalid => {
                if let Some(toe) = self.toe_gpst(t) {
                    if t > toe {
                        self.state = State::Valid;
                    } else {
                        let dt = (t - toe).abs();
                        debug!("{} - TOE: {} - {} to go", t, toe, dt);
                    }
                } else {
                    self.state = State::Collecting;
                }
            },
            State::Valid => {
                if let Some(_) = self.toe_gpst(t) {
                } else {
                    self.state = State::Collecting;
                }
            },
        }
    }

    fn is_complete(&self) -> bool {
        if let Some(frame1) = self.frame1.as_ref() {
            if let Some(frame2) = self.frame2.as_ref() {
                if let Some(frame3) = self.frame3.as_ref() {
                    if frame2.iode == frame3.iode {
                        if frame1.iodc as u8 == frame2.iode {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    fn week_number(t_gpst: Epoch, wn: u16) -> u32 {
        let current_week = t_gpst.to_time_of_week().0;
        let delta = current_week - wn as u32;
        let rollover = (delta as f64 / 1024.0).round() as u32;
        wn as u32 + rollover * 1024
    }

    pub fn to_kepler(&self, t_gpst: Epoch) -> Option<SVKepler> {
        if self.is_complete() {
            return None;
        }

        let (frame1, frame2, frame3) = (
            self.frame1.as_ref()?,
            self.frame2.as_ref()?,
            self.frame3.as_ref()?,
        );

        let toc_gpst = self.toc_gpst(t_gpst)?;
        let toe_gpst = self.toe_gpst(t_gpst)?;

        if toe_gpst < toc_gpst {
            Some(SVKepler::from_gps(
                self.sv,
                frame2.toe_s as f64,
                toc_gpst,
                toe_gpst,
                frame1,
                frame2,
                frame3,
            ))
        } else {
            None
        }
    }

    fn toc_gpst(&self, t_gpst: Epoch) -> Option<Epoch> {
        let frame1 = self.frame1.as_ref()?;

        let toc_nanos = (frame1.toc_s as u64) * 1_000_000_000;
        let week = Self::week_number(t_gpst, frame1.week);

        Some(Epoch::from_time_of_week(week, toc_nanos, TimeScale::GPST))
    }

    fn toe_gpst(&self, t_gpst: Epoch) -> Option<Epoch> {
        let (frame1, frame2) = (self.frame1.as_ref()?, self.frame2.as_ref()?);

        let toe_nanos = (frame2.toe_s as u64) * 1_000_000_000;
        let week = Self::week_number(t_gpst, frame1.week);

        Some(Epoch::from_time_of_week(week, toe_nanos, TimeScale::GPST))
    }

    pub fn clock_correction(&self, t: Epoch) -> Option<ClockCorrection> {
        if self.is_complete() {
            return None;
        }

        let frame1 = self.frame1.as_ref()?;
        let t_gpst = t.to_time_scale(TimeScale::GPST);
        let (a0, a1, a2) = (frame1.af0_s, frame1.af1_s_s, frame1.af2_s_s2);

        let toc_gpst = self.toc_gpst(t_gpst)?;
        let mut dt = (t_gpst - toc_gpst).to_seconds();

        for _ in 0..10 {
            dt -= a0 + a1 * dt + a2 * dt.powi(2);
        }

        let dt = Duration::from_seconds(a0 + a1 * dt + a2 * dt.powi(2));
        Some(ClockCorrection::without_relativistic_correction(dt))
    }
}

#[derive(Default)]
pub struct EphemerisBuffer {
    pub buffer: Vec<GpsSvEphemeris>,
}

impl EphemerisBuffer {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(16),
        }
    }

    pub fn update(&mut self, t: Epoch, sv: SV, frame: RxmSfrbxGpsQzssFrame) {
        if let Some(sv_data) = self.buffer.iter_mut().find(|buf| buf.sv == sv) {
            sv_data.update(t, frame.subframe);
        } else {
            let mut new = GpsSvEphemeris::default();
            new.sv = sv;

            match frame.subframe {
                RxmSfrbxGpsQzssSubframe::Eph1(eph1) => {
                    new.frame1 = Some(eph1);
                },
                RxmSfrbxGpsQzssSubframe::Eph2(eph2) => {
                    new.frame2 = Some(eph2);
                },
                RxmSfrbxGpsQzssSubframe::Eph3(eph3) => {
                    new.frame3 = Some(eph3);
                },
            }

            self.buffer.push(new);
        }
    }

    pub fn clock_correction(&self, t: Epoch, sv: SV) -> Option<ClockCorrection> {
        if let Some(buff) = self
            .buffer
            .iter()
            .find(|buf| buf.sv == sv && buf.is_complete())
        {
            buff.clock_correction(t)
        } else {
            None
        }
    }
}
