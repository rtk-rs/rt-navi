use gnss_rtk::prelude::{
    Bias,
    BiasRuntime, //IonosphereModel, KbModel
    TroposphereModel,
};

pub struct BiasModels {}

impl Bias for BiasModels {
    fn troposphere_bias_m(&self, rtm: &BiasRuntime) -> f64 {
        TroposphereModel::Niel.bias_m(rtm)
    }

    fn ionosphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
        // IonosphereModel::KbModel(KbModel {
        //     alpha: (0.0, 0.0, 0.0, 0.0),
        //     beta: (0.0, 0.0, 0.0, 0.0),
        //     h_km: 0.0,
        // })
    }
}
