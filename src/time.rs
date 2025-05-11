use gnss_rtk::prelude::{AbsoluteTime, Epoch, TimeScale};

pub struct Time {}

impl AbsoluteTime for Time {
    fn new_epoch(&mut self, _: Epoch) {}

    fn epoch_correction(&self, t: Epoch, target: TimeScale) -> Epoch {
        t.to_time_scale(target)
    }
}
