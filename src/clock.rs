use gnss_rtk::prelude::{ClockCorrection, Duration, SV};

#[derive(Debug, Copy, Clone)]
pub struct SvClock {
    pub sv: SV,
    pub af0: f64,
}

impl SvClock {
    pub fn new(sv: SV, af0: f64) -> Self {
        Self { sv, af0 }
    }
}

pub struct ClockBuffer {
    /// storage
    pub buffer: Vec<SvClock>,
}

impl ClockBuffer {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(8),
        }
    }

    pub fn latch(&mut self, clock: SvClock) {
        self.buffer.retain(|buf| buf.sv != clock.sv);
        self.buffer.push(clock);
    }

    pub fn clock_correction(&self, sv: SV) -> Option<ClockCorrection> {
        let buf = self.buffer.iter().find(|buf| buf.sv == sv)?;
        let dt = Duration::from_seconds(buf.af0);
        Some(ClockCorrection::without_relativistic_correction(dt))
    }
}
