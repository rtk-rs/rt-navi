//! High precision navigation, in real time

// private
mod cli;
mod ublox;

use env_logger::{Builder, Target};

#[macro_use]
extern crate log;

use cli::Cli;
use thiserror::Error;

use gnss_rtk::prelude::{
    Config, Error as RTKError, InvalidationCause, IonosphereBias, Method, Solver, TroposphereBias,
};

use tokio::sync::mpsc;
use ublox::{Command, Message, Ublox};

#[derive(Debug, Error)]
pub enum Error {
    #[error("non supported constellation {0}")]
    NonSupportedGnss(u8),
    #[error("non supported signal {0}")]
    NonSupportedSignal(u8),
}

#[tokio::main]
async fn main() -> Result<(), Error> {
    let mut builder = Builder::from_default_env();
    builder
        .target(Target::Stdout)
        .format_timestamp_secs()
        .format_module_path(false)
        .init();

    // cli and user args
    let cli = Cli::new();
    let opts = cli.serial_opts();

    // create channels
    let (ublox_tx, mut rx) = mpsc::channel(16);
    let (tx, mut ublox_rx) = mpsc::channel(16);

    let method = Method::SPP;
    let cfg = Config::static_preset(method);

    let mut solver = Solver::new(&cfg, None, |t, sv, _| None)
        .unwrap_or_else(|e| panic!("failed to deploy solver: {}", e));

    // deploy hardware
    let mut ublox = Ublox::new(opts, ublox_rx, ublox_tx);
    ublox.init();
    tokio::spawn(async move {
        ublox.tasklet();
    });

    let mut ionod = IonosphereBias::default();
    let mut tropod = TroposphereBias::default();

    loop {
        while let Some(msg) = rx.recv().await {
            match msg {
                Message::Candidates((t, candidates)) => {
                    match solver.resolve(t, &candidates, &ionod, &tropod) {
                        Ok((_, solution)) => {
                            let (x, y, z) = (
                                solution.position.x,
                                solution.position.y,
                                solution.position.z,
                            );
                            let (vel_x, vel_y, vel_z) = (
                                solution.velocity.x,
                                solution.velocity.y,
                                solution.velocity.z,
                            );
                            let dt = solution.dt;
                            info!("new solution");
                            info!("x={}, y={}, z={}", x, y, z);
                            info!("vel_x={}, vel_y={}, vel_z={}", vel_x, vel_y, vel_z);
                            info!("dt={}", dt);
                        },
                        Err(e) => match e {
                            RTKError::Almanac(e) => {
                                panic!("failed to load latest almanac: {}", e);
                            },
                            RTKError::NotEnoughCandidates => {
                                error!("not enough candidates");
                            },
                            RTKError::NotEnoughMatchingCandidates => {
                                error!("not enough quality candidates");
                            },
                            RTKError::MatrixError
                            | RTKError::NavigationError
                            | RTKError::MatrixInversionError => {
                                error!("navigation error");
                                warn!("check configuration setup");
                            },
                            RTKError::MissingPseudoRange | RTKError::PseudoRangeCombination => {
                                error!("missing pseudo range observation");
                            },
                            RTKError::PhaseRangeCombination => {
                                error!("missing pseudo range observation");
                            },
                            RTKError::UnresolvedState => {
                                error!("solver internal error");
                            },
                            RTKError::UnresolvedAmbiguity => {
                                error!("solver internal error (ambiguity)");
                            },
                            RTKError::InvalidStrategy => error!("invalid solving strategy"),
                            RTKError::BancroftError => {
                                error!("bancroft error");
                                warn!("check configuration setup");
                            },
                            RTKError::BancroftImaginarySolution => {
                                error!("imaginary solution");
                                warn!("check configuration setup");
                            },
                            RTKError::FirstGuess => {
                                error!("first guess error");
                                warn!("check configuration setup");
                            },
                            RTKError::TimeIsNan => {
                                error!("resolved time is NaN");
                                warn!("check configuration setup");
                            },
                            RTKError::PhysicalNonSenseRxPriorTx
                            | RTKError::PhysicalNonSenseRxTooLate => {
                                error!("physical non sense");
                                warn!("check configuration setup");
                            },
                            RTKError::Physics(e) => {
                                error!("physical non sense: {}", e);
                                warn!("check configuration setup");
                            },
                            RTKError::InvalidatedSolution(cause) => match cause {
                                InvalidationCause::FirstSolution => {
                                    info!("first fix is pending!");
                                },
                                InvalidationCause::GDOPOutlier(gdop) => {
                                    error!("solution rejected: gdop={}", gdop);
                                },
                                InvalidationCause::TDOPOutlier(tdop) => {
                                    error!("solution rejected: tdop={}", tdop);
                                },
                                InvalidationCause::InnovationOutlier(innov) => {
                                    error!("solution rejected: innov={}", innov);
                                },
                                InvalidationCause::CodeResidual(code_res) => {
                                    error!("solution rejected: code_res={}", code_res);
                                },
                            },
                        },
                    }
                },
            }
        }
    }
}
