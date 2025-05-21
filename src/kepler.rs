use anise::math::Matrix3;
use log::debug;
use std::{cell::RefCell, f64::consts::PI};

use gnss_rtk::prelude::{
    Duration, Epoch, Frame, Orbit, OrbitSource, TimeScale, SPEED_OF_LIGHT_M_S, SV,
};

use gnss_protos::{GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3};

use crate::ephemeris::GpsSvEphemeris;

use nalgebra::{Rotation3, Vector3};

#[derive(Debug, Copy, Clone)]
pub struct SVKepler {
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
    week: u16,
    omega: f64,
    omega0: f64,
    omega_dot: f64,
    toc_gpst: Epoch,
    toe_gpst: Epoch,
    toe_s: f64,
}

impl SVKepler {
    pub fn from_gps(
        sv: SV,
        toe_s: f64,
        toc_gpst: Epoch,
        toe_gpst: Epoch,
        frame1: &GpsQzssFrame1,
        frame2: &GpsQzssFrame2,
        frame3: &GpsQzssFrame3,
    ) -> Self {
        Self {
            sv,
            toe_s,
            toc_gpst,
            toe_gpst,
            e: frame2.e,
            cuc: frame2.cuc,
            cus: frame2.cus,
            crc: frame3.crc,
            crs: frame2.crs,
            cic: frame3.cic,
            cis: frame3.cis,
            week: frame1.week,
            m0: frame2.m0 * PI,
            i0: frame3.i0 * PI,
            i_dot: frame3.idot * PI,
            delta_n: frame2.dn * PI,
            a: frame2.sqrt_a.powi(2),
            omega: frame3.omega * PI,
            omega0: frame3.omega0 * PI,
            omega_dot: frame3.omega_dot * PI,
        }
    }
}

#[derive(Clone)]
pub struct KeplerBuffer {
    /// storage
    buffer: RefCell<Vec<SVKepler>>,
}

impl KeplerBuffer {
    pub fn new() -> Self {
        Self {
            buffer: RefCell::new(Vec::with_capacity(8)),
        }
    }

    pub fn latch(&self, kepler: SVKepler) {
        self.buffer.borrow_mut().retain(|buf| buf.sv != kepler.sv);
        self.buffer.borrow_mut().push(kepler);
    }
}

impl OrbitSource for KeplerBuffer {
    fn next_at(&self, epoch: Epoch, sv: SV, frame: Frame) -> Option<Orbit> {
        debug!("{} request @ {}", sv, epoch);

        let buffer = self.buffer.borrow();
        let sv_data = buffer.iter().find(|buf| buf.sv == sv)?;

        // constants
        const GM_M3_S2: f64 = 3.986004418E14;
        const OMEGA_EARTH: f64 = 7.2921151467E-5;

        let (e, e_2) = (sv_data.e, sv_data.e.powi(2));
        let (a, a_3) = (sv_data.a, sv_data.a.powi(3));

        let (cus, cuc) = (sv_data.cus, sv_data.cuc);
        let (cis, cic) = (sv_data.cis, sv_data.cic);
        let (crs, crc) = (sv_data.crs, sv_data.crc);

        let (i0, idot) = (sv_data.i0, sv_data.i_dot);
        let (omega0, omega, omega_dot) = (sv_data.omega0, sv_data.omega, sv_data.omega_dot);

        let t_gpst = epoch.to_time_scale(TimeScale::GPST);

        let t_k = (t_gpst - sv_data.toe_gpst).to_seconds();
        debug!("{}({}) - toe={} t_k={}", t_gpst, sv, sv_data.toe_gpst, t_k);

        let n0 = (GM_M3_S2 / a_3).sqrt();
        let n = n0 + sv_data.delta_n;
        let m = sv_data.m0 + n * t_k;

        let mut e_k = 0.0_f64;
        let mut e_k_lst = 0.0_f64;

        loop {
            e_k = m + e * e_k_lst.sin();
            if (e_k - e_k_lst).abs() < 1e-10 {
                break;
            }

            e_k_lst = e_k;
        }

        let (sin_e_k, cos_e_k) = e_k.sin_cos();
        let v_k = ((1.0 - e_2).sqrt() * sin_e_k).atan2(cos_e_k - e);
        let (sin_v_k, cos_v_k) = v_k.sin_cos();

        let phi = v_k + omega;
        let (sin_2phi, cos_2phi) = (2.0 * phi).sin_cos();

        let u_k = phi + cuc * cos_2phi + cus * sin_2phi;
        let r_k = a * (1.0 - e * cos_e_k) + crc * cos_2phi + crs * sin_2phi;
        let i_k = i0 + idot * t_k + cic * cos_2phi + cis * sin_2phi;
        let omega_k = omega0 + (omega_dot - OMEGA_EARTH) * t_k - OMEGA_EARTH * sv_data.toe_s;

        let (x, y, z) = (r_k * u_k.cos(), r_k * u_k.sin(), 0.0);

        // MEO orbit to ECEF rotation matrix
        let rot_x3 = Rotation3::from_axis_angle(&Vector3::x_axis(), i_k);
        let rot_z3 = Rotation3::from_axis_angle(&Vector3::z_axis(), omega_k);
        let rot3 = rot_z3 * rot_x3;

        let xyz_vec3 = Vector3::new(x, y, z);
        let xyz_ecef = rot3 * xyz_vec3;

        let (x_km, y_km, z_km) = (
            xyz_ecef[0] / 1000.0,
            xyz_ecef[1] / 1000.0,
            xyz_ecef[2] / 1000.0,
        );

        debug!(
            "{}({}) a={}m tk={}s n={}rad/s m={}rad e={} e_k={}rad r={} i={}",
            t_gpst, sv, a, t_k, n, m, e, e_k, r_k, i_k
        );

        debug!("{}({}) x={}km y={}km z={}km", t_gpst, sv, x_km, y_km, z_km);

        Some(Orbit::from_position(x_km, y_km, z_km, t_gpst, frame))
    }
}
