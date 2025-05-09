use log::{debug, error};
use std::cell::RefCell;

use ublox::MgaGpsEphRef;

use gnss_rtk::prelude::{Epoch, Frame, Orbit, OrbitSource, TimeScale, SV};

#[derive(Debug, Copy, Clone)]
pub struct SvKepler {
    pub sv: SV,
    a: f64,
    e: f64,
    m0: f64,
    i0: f64,
    cuc: f64,
    cus: f64,
    crc: f64,
    crs: f64,
    cic: f64,
    cis: f64,
    i_dot: f64,
    delta_n: f64,
    omega: f64,
    omega0: f64,
    omega_dot: f64,
    toc: f64,
    toe: f64,
}

impl SvKepler {
    pub fn new(sv: SV, value: MgaGpsEphRef) -> Self {
        Self {
            sv,
            e: value.e(),
            m0: value.m0(),
            cuc: value.cuc(),
            cus: value.cus(),
            cic: value.cic(),
            cis: value.cis(),
            crc: value.crc(),
            crs: value.crs(),
            i0: value.i0(),
            toc: value.toc(),
            toe: value.toe(),
            i_dot: value.idot(),
            a: value.sqrt_a().powi(2),
            delta_n: value.delta_n(),
            omega: value.omega(),
            omega0: value.omega(),
            omega_dot: value.omega_dot(),
        }
    }
}

#[derive(Clone)]
pub struct KeplerBuffer {
    /// storage
    buffer: RefCell<Vec<SvKepler>>,
}

impl KeplerBuffer {
    pub fn new() -> Self {
        Self {
            buffer: RefCell::new(Vec::with_capacity(8)),
        }
    }

    pub fn latch(&self, kepler: SvKepler) {
        self.buffer.borrow_mut().retain(|buf| buf.sv != kepler.sv);
        self.buffer.borrow_mut().push(kepler);
    }
}

impl OrbitSource for KeplerBuffer {
    fn next_at(&self, epoch: Epoch, sv: SV, frame: Frame) -> Option<Orbit> {
        let buffer = self.buffer.borrow();
        let sv_data = buffer.iter().find(|buf| buf.sv == sv)?;

        let gm_m3_s2 = 0.0_f64; // Constants::gm(sv);
        let omega = 0.0_f64; // Constants::omega(sv);
        let dtr_f = 0.0_f64; // Constants::dtr_f(sv);

        let t_gpst = epoch.to_time_scale(TimeScale::GPST);
        let (_, t_gpst) = t_gpst.to_time_of_week();
        let t_k = t_gpst as f64 - sv_data.toe;

        let n0 = (gm_m3_s2 / sv_data.a.powi(3)).sqrt(); // average angular vel
        let n = n0 + sv_data.delta_n; // corrected mean angular vel
        let m_k = sv_data.m0 + n * t_k; // average anomaly

        let mut e_k;
        let mut i = 0;
        let mut e_k_lst = 0.0_f64;

        loop {
            i += 1;
            e_k = m_k + sv_data.e * e_k_lst.sin();

            if (e_k - e_k_lst).abs() < 1e-10 {
                break;
            }

            if i > 10 {
                break; //ERROR
            }

            e_k_lst = e_k;
        }

        if i == 11 {
            return None; //ERROR
        }

        // true anomaly
        let (sin_e_k, cos_e_k) = e_k.sin_cos();
        let v_k = ((1.0 - sv_data.e.powi(2)).sqrt() * sin_e_k).atan2(cos_e_k - sv_data.e);

        let phi_k = v_k + sv_data.omega; // latitude argument
        let (x2_sin_phi_k, x2_cos_phi_k) = (2.0 * phi_k).sin_cos();

        // latitude argument correction
        let du_k = sv_data.cus * x2_sin_phi_k + sv_data.cuc * x2_cos_phi_k;
        let u_k = phi_k + du_k;

        // orbital radius correction
        let dr_k = sv_data.crs * x2_sin_phi_k + sv_data.crc * x2_cos_phi_k;
        let r_k = sv_data.a * (1.0 - sv_data.e * e_k.cos()) + dr_k;

        // inclination angle correction
        let di_k = sv_data.cis * x2_sin_phi_k + sv_data.cic * x2_cos_phi_k;

        // first derivatives
        let fd_omega_k = sv_data.omega_dot - omega;

        let fd_e_k = n / (1.0 - sv_data.e * e_k.cos());

        let fd_phi_k = ((1.0 + sv_data.e) / (1.0 - sv_data.e)).sqrt()
            * ((v_k / 2.0).cos() / (e_k / 2.0).cos()).powi(2)
            * fd_e_k;

        let fd_u_k =
            (sv_data.cus * x2_cos_phi_k - sv_data.cuc * x2_sin_phi_k) * fd_phi_k * 2.0 + fd_phi_k;

        let fd_r_k = sv_data.a * sv_data.e * e_k.sin() * fd_e_k
            + 2.0 * (sv_data.crs * x2_cos_phi_k - sv_data.crc * x2_sin_phi_k) * fd_phi_k;

        let fd_i_k = sv_data.i_dot
            + 2.0 * (sv_data.cis * x2_cos_phi_k - sv_data.crc * x2_sin_phi_k) * fd_phi_k;

        // relativistic effect correction
        let dtr = dtr_f * sv_data.e * sv_data.a.sqrt() * e_k.sin();
        let fd_dtr = dtr_f * sv_data.e * sv_data.a.sqrt() * e_k.cos() * fd_e_k;

        // ascending node longitude correction (RAAN)
        let omega_k = if sv.is_beidou_geo() {
            sv_data.omega0 + sv_data.omega_dot * t_k - omega * sv_data.toe
        } else {
            // GPS, Galileo, BeiDou_meo
            (sv_data.omega0 + sv_data.omega_dot - omega) * t_k - omega * sv_data.toe
        };

        // corrected inclination angle
        let i_k = sv_data.i0 + di_k + sv_data.i_dot * t_k;

        match Orbit::try_keplerian(
            sv_data.a.sqrt() * 1e-3,
            sv_data.e,
            i_k.to_degrees(),
            sv_data.omega_dot.to_degrees(),
            sv_data.omega.to_degrees(),
            v_k.to_degrees(),
            epoch,
            frame.with_mu_km3_s2(gm_m3_s2 * 1e-9),
        ) {
            Ok(orbit) => {
                let posvel_km = orbit.to_cartesian_pos_vel();

                debug!(
                    "(anise) {}({}) x_km={}, y_km={} z_km={}",
                    epoch, sv, posvel_km[0], posvel_km[1], posvel_km[2]
                );

                Some(orbit)
            },
            Err(e) => {
                error!("(anise) kepler error: {}", e);
                None
            },
        }
    }
}
