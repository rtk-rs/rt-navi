use log::{debug, error};
use std::{cell::RefCell, f64::consts::PI};

use ublox::{GpsEphFrame1, GpsEphFrame2, GpsEphFrame3};

use gnss_rtk::prelude::{
    Constellation, Epoch, Frame, Orbit, OrbitSource, TimeScale, EARTH_J2000, SPEED_OF_LIGHT_M_S, SV,
};

use nalgebra::{Rotation3, Vector3};

use crate::ephemeris::{EphemerisBuffer, GpsSvRawEphemeris};

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
    toc: u32,
    toe: u32,
    i_dot: f64,
    delta_n: f64,
    week: u16,
    omega: f64,
    omega0: f64,
    omega_dot: f64,
}

impl SVKepler {
    pub fn from_gps(
        sv: SV,
        frame1: &GpsEphFrame1,
        frame2: &GpsEphFrame2,
        frame3: &GpsEphFrame3,
    ) -> Self {
        Self {
            sv,
            a: frame2.sqrt_a.powi(2),
            e: frame2.e,
            m0: frame2.m0,
            i0: frame2.m0,
            cuc: frame2.cuc,
            cus: frame2.cus,
            crc: frame3.crc,
            crs: frame2.crs,
            cic: frame3.cic,
            cis: frame3.cis,
            i_dot: frame3.idot,
            week: frame1.week,
            toc: frame1.toc,
            toe: frame2.toe,
            delta_n: frame2.delta_n,
            omega: frame3.omega,
            omega0: frame3.omega0,
            omega_dot: frame3.omega_dot,
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
        let buffer = self.buffer.borrow();
        let sv_data = buffer.iter().find(|buf| buf.sv == sv)?;

        // constants
        let gm_m3_s2 = 3.986005_f64 * 10.0_f64.powi(14);
        let omega_earth = 7.2921151467_f64.powi(-5);
        const PI2: f64 = PI * 2.0;

        // values
        let a_ref = 26559710.0_f64; // almanach
        let f = -2.0 * gm_m3_s2.sqrt() / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S;
        let (toc, toe) = (sv_data.toc, sv_data.toe);

        let (e, e_2, e_3) = (sv_data.e, sv_data.e.powi(2), sv_data.e.powi(3));
        let (a, sqrt_a, a_3) = (sv_data.a, sv_data.a.sqrt(), sv_data.a.powi(3));

        let (cus, cuc) = (sv_data.cus, sv_data.cuc);
        let (cis, cic) = (sv_data.cis, sv_data.cic);
        let (crs, crc) = (sv_data.crs, sv_data.crc);

        let (i0, idot) = (sv_data.i0, sv_data.i_dot);
        let (omega0, omega_dot) = (sv_data.omega0, sv_data.omega_dot);

        let toe_s = sv_data.toe as f64;

        // dt
        let t_gpst = epoch.to_time_scale(TimeScale::GPST);

        let toe_wn = GpsSvRawEphemeris::week_number(t_gpst, sv_data.week);
        let toe_nanos = (sv_data.toe as u64) * 1_000_000_000;
        let toe_gpst = Epoch::from_time_of_week(toe_wn, toe_nanos, TimeScale::GPST);

        let mut t_k = (t_gpst - toe_gpst).to_seconds();

        if t_k > 302400.0 {
            t_k -= 604800.0;
        } else if t_k < -302400.0 {
            t_k += 604800.0;
        }

        let n0 = (gm_m3_s2 / a_3).sqrt();
        let n = n0 + sv_data.delta_n;

        let mut m = sv_data.m0 + n * t_k;

        let mut e_k = 0.0_f64;
        let mut e_k_lst = 0.0_f64;

        for _ in 0..10 {
            e_k = m + e * e_k_lst.sin();
            e_k_lst = e_k;
        }

        let (sin_e_k, cos_e_k) = e_k.sin_cos();
        let v_k = ((1.0 - e_2).sqrt() * sin_e_k).atan2(cos_e_k - e);

        let phi = sv_data.omega + v_k;
        let (sin_2phi, cos_2phi) = (2.0 * phi).sin_cos();

        let du = cus * sin_2phi + cuc * cos_2phi;
        let u = phi + du;

        let dr = crs * sin_2phi + crc * cos_2phi;
        let r = a * (1.0 - e * cos_e_k) + dr;

        let di = cis * sin_2phi + cic * cos_2phi;
        let i = i0 + di + idot * t_k;

        let (x_orb, y_orb) = (r * u.cos(), r * u.sin());
        let omega = omega0 + (omega_dot - omega_earth) * t_k - omega_earth * toe_s;

        let (sin_i, cos_i) = i.sin_cos();
        let (sin_omega, cos_omega) = omega.sin_cos();

        let x = x_orb * cos_omega - y_orb * cos_i * sin_omega;
        let y = x_orb * sin_omega - y_orb * cos_i * cos_omega;
        let z = 0.0_f64;

        let rotation_x = Rotation3::from_axis_angle(&Vector3::x_axis(), i);
        let rotation_z = Rotation3::from_axis_angle(&Vector3::z_axis(), omega);
        let rot3 = rotation_x * rotation_z;
        let orbit_xyz = Vector3::new(x, y, z);
        let ecef_xyz = rot3 * orbit_xyz;

        let (x_km, y_km, z_km) = (
            ecef_xyz[0] / 1000.0,
            ecef_xyz[1] / 1000.0,
            ecef_xyz[2] / 1000.0,
        );

        debug!(
            "{}({}) a={}m tk={}s n={}rad/s m={}rad e={} e_k={}rad r={} i={}",
            t_gpst, sv, a, t_k, n, m, e, e_k, r, i
        );

        debug!("{}({}) x={}km y={}km z={}km", t_gpst, sv, x_km, y_km, z_km);

        Some(Orbit::from_position(x_km, y_km, z_km, t_gpst, frame))
    }
}
