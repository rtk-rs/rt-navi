use gnss_rtk::prelude::{ClockCorrection, Duration, SV};

#[derive(Debug, Copy, Clone)]
pub struct SvClock {
    pub sv: SV,
    pub af0: f64,
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
        if let Some(sv_clk) = self.buffer.iter_mut().find(|buf| buf.sv == clock.sv) {
            sv_clk.af0 = clock.af0;
        } else {
            self.buffer.push(clock);
        }
    }

    pub fn clock_correction(&self, sv: SV) -> Option<ClockCorrection> {
        let buf = self.buffer.iter().find(|buf| buf.sv == sv)?;
        let dt = Duration::from_seconds(buf.af0);
        Some(ClockCorrection::without_relativistic_correction(dt))
    }
}
