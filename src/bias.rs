use gnss_rtk::prelude::{Bias, BiasRuntime};

pub struct BiasModels {}

impl Bias for BiasModels {
    fn troposphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
    }

    fn ionosphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
    }
}
