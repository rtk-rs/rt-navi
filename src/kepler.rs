use log::{debug, error};
use std::cell::RefCell;

use ublox::{GpsEphFrame1, GpsEphFrame2, GpsEphFrame3};

use gnss_rtk::prelude::{
    Constellation, Epoch, Frame, Orbit, OrbitSource, TimeScale, SPEED_OF_LIGHT_M_S, SV,
};

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
    toc: Epoch,
    toe: Epoch,
    i_dot: f64,
    delta_n: f64,
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
            delta_n: frame2.delta_n,
            omega: frame3.omega,
            omega0: frame3.omega0,
            omega_dot: frame3.omega_dot,
            toc: {
                let toc_nanos = (frame1.toc as u64) * 1_000_000_000;
                Epoch::from_time_of_week(frame1.week as u32, toc_nanos, TimeScale::GPST)
            },
            toe: {
                let toe_nanos = (frame2.toe as u64) * 1_000_000_000;
                Epoch::from_time_of_week(frame1.week as u32, toe_nanos, TimeScale::GPST)
            },
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

        let toe_nanos = sv_data.toe.to_time_of_week().1;
        let toe_s = (toe_nanos as f64) * 1.0E-9;

        // dt
        let t_gpst = epoch.to_time_scale(TimeScale::GPST);
        let toe_gpst = sv_data.toe.to_time_scale(TimeScale::GPST);
        let t_k = (t_gpst - toe_gpst).to_seconds();

        let n0 = (gm_m3_s2 / a_3).sqrt();
        let n = n0 + sv_data.delta_n;
        let m = sv_data.m0 + n * t_k;

        let e0 = m;
        let mut e_k = e0;

        for _ in 0..10 {
            e_k = m + e * e_k.sin();
        }

        let (sin_e_k, cos_e_k) = e_k.sin_cos();
        let v_k = ((1.0 - e_2).sqrt() * sin_e_k).atan2(cos_e_k - e);

        let phi = sv_data.omega + v_k;
        let (sin_2phi, cos_2phi) = (2.0 * phi).sin_cos();

        let u = phi + cus * sin_2phi + cuc * cos_2phi;
        let dr = crs * sin_2phi + crc * cos_2phi;
        let r = a * (1.0 - e * cos_e_k) + dr;
        let i = i0 + idot * t_k + cis * sin_2phi + cic * cos_2phi;

        let (x_orb, y_orb) = (r * u.cos(), r * u.sin());
        let omega = omega0 + (omega_dot - omega_earth) * t_k - omega_earth * toe_s;

        let (sin_i, cos_i) = i.sin_cos();
        let (sin_omega, cos_omega) = omega.sin_cos();

        let x = x_orb * cos_omega - y_orb * cos_i * sin_omega;
        let y = x_orb * sin_omega - y_orb * cos_i * cos_omega;
        let z = y_orb + sin_i;

        let (x_km, y_km, z_km) = (x / 1000.0, y / 1000.0, z / 1000.0);

        debug!("{}({}) x={}km y={}km z={}km", t_gpst, sv, x_km, y_km, z_km);

        Some(Orbit::from_position(x_km, y_km, z_km, t_gpst, frame))
    }
}
