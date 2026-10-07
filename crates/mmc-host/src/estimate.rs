//! The online R/ψ estimator ([`mmc_core::estim`]) on the host: the same
//! block-mean estimator the drive runs, over any capture that recorded the
//! dq channels ([`estimate_file`]), or fed frame by frame by a live tool
//! ([`EstRun`]).

use std::io::Write as _;
use std::path::Path;

use mmc_core::estim::{RpsiAverager, RpsiCfg, RpsiEstimator};
use mmc_core::transforms::Dq;
use mmc_proto::channel;

/// Block length the estimator averages over [s]: long against the current
/// loop, short against the i_d dither.
pub const BLOCK: f32 = 0.05;

/// Estimator tuning per 50 ms block: R may drift ~1 %/s and ψ ~0.1 %/s;
/// ~10 mV of voltage error per averaged row.
pub fn estim_cfg(l: f32) -> RpsiCfg {
    RpsiCfg {
        ld: l,
        lq: l,
        q_r: 1e-5,
        q_psi: 2e-12,
        // The d-axis offset moves with the operating point (dead-time residue
        // depends on current and duty), but it must move slowly against the
        // i_d dither or it absorbs the steps R is read from.
        q_bias: 1e-6,
        noise: 1e-4,
        omega_min: 100.0,
        i_min: 0.1,
        use_derivative: false,
        p0: 1.0,
    }
}

/// One estimator trajectory point.
#[derive(Clone, Debug)]
pub struct EstPoint {
    pub t: f64,
    pub r: f32,
    pub psi: f32,
    pub sigma_r: f32,
    pub sigma_psi: f32,
    pub bias_d: f32,
    pub res_d: f32,
    pub res_q: f32,
}

/// Feeds the frames of one source through an estimator.
pub struct EstRun {
    pub avg: RpsiAverager,
    last_t: Option<f64>,
    pub points: Vec<EstPoint>,
}

impl EstRun {
    pub fn new(cfg: RpsiCfg, r0: f32, psi0: f32) -> Self {
        Self {
            avg: RpsiAverager::new(RpsiEstimator::new(cfg, r0, psi0), BLOCK),
            last_t: None,
            points: Vec::new(),
        }
    }

    pub fn est(&self) -> &RpsiEstimator {
        &self.avg.est
    }

    /// `vals` are the frame's values in `MASK` order (or any mask, with
    /// `col` mapping channel → column).
    pub fn push(&mut self, t: f64, get: impl Fn(u8) -> Option<f32>) {
        let (Some(id), Some(iq), Some(vd), Some(vq), Some(w), Some(st)) = (
            get(channel::I_D),
            get(channel::I_Q),
            get(channel::V_D),
            get(channel::V_Q),
            get(channel::OMEGA_HALL),
            get(channel::STATE),
        ) else {
            return;
        };
        let dt = self.last_t.map(|l| (t - l) as f32).unwrap_or(0.0);
        self.last_t = Some(t);
        let running = st as u8 == mmc_drive::ST_RUN;
        if !self
            .avg
            .push(Dq { d: vd, q: vq }, Dq { d: id, q: iq }, w, dt, running)
        {
            return;
        }
        let e = &self.avg.est;
        let (sr, sp) = e.sigma();
        self.points.push(EstPoint {
            t,
            r: e.r(),
            psi: e.psi(),
            sigma_r: sr,
            sigma_psi: sp,
            bias_d: e.bias_d(),
            res_d: e.residual.d,
            res_q: e.residual.q,
        });
    }

    pub fn write_csv(&self, out: &Path) -> std::io::Result<()> {
        let mut w = std::io::BufWriter::new(std::fs::File::create(out)?);
        writeln!(w, "t,r_hat,psi_hat,sigma_r,sigma_psi,bias_d,res_d,res_q")?;
        for p in &self.points {
            writeln!(
                w,
                "{},{},{},{},{},{},{},{}",
                p.t, p.r, p.psi, p.sigma_r, p.sigma_psi, p.bias_d, p.res_d, p.res_q
            )?;
        }
        w.flush()
    }

    /// Mean of the estimates over the last `secs` of running.
    pub fn settled(&self, secs: f64) -> Option<(f32, f32)> {
        let t_end = self.points.last()?.t;
        let tail: Vec<&EstPoint> = self.points.iter().filter(|p| p.t >= t_end - secs).collect();
        let n = tail.len() as f32;
        Some((
            tail.iter().map(|p| p.r).sum::<f32>() / n,
            tail.iter().map(|p| p.psi).sum::<f32>() / n,
        ))
    }
}

/// Run the estimator over a capture CSV; write `<stem>.estim.csv` beside it
/// in `out_dir` and return the settled `(R, ψ)`.
pub fn estimate_file(
    csv: &Path,
    r0: f32,
    psi0: f32,
    l: f32,
    out_dir: &Path,
) -> std::io::Result<Option<(f32, f32)>> {
    let text = std::fs::read_to_string(csv)?;
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap_or("").split(',').collect();
    let col = |ch: u8| {
        header
            .iter()
            .position(|h| *h == channel::NAMES[ch as usize])
    };
    let cols: Vec<(u8, usize)> = (0..channel::COUNT as u8)
        .filter_map(|c| col(c).map(|i| (c, i)))
        .collect();
    let mut run = EstRun::new(estim_cfg(l), r0, psi0);
    for line in lines {
        let v: Vec<f64> = line
            .split(',')
            .map(|x| x.parse().unwrap_or(f64::NAN))
            .collect();
        let get = |ch: u8| {
            cols.iter()
                .find(|(c, _)| *c == ch)
                .map(|(_, i)| v[*i] as f32)
        };
        run.push(v[0], get);
    }
    std::fs::create_dir_all(out_dir)?;
    let stem = csv.file_stem().unwrap().to_string_lossy();
    run.write_csv(&out_dir.join(format!("{stem}.estim.csv")))?;
    Ok(run.settled(1.0))
}
