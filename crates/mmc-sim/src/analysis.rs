//! Step-response metrics for regression tests and the host CLI.

#[derive(Copy, Clone, Debug)]
pub struct StepMetrics {
    /// 10%–90% rise time [s].
    pub rise_time: f32,
    /// Peak overshoot beyond the target, as a fraction of the step (0.05 = 5%).
    pub overshoot: f32,
    /// Mean error over the last 20% of the trace, as a fraction of the step.
    pub steady_state_error: f32,
}

/// Analyze a step response `y(t)` toward `target`, assuming the step is
/// applied at `t[0]` from y ≈ 0. Returns `None` if the trace is too short or
/// never crosses the 10%/90% thresholds.
pub fn step_metrics(t: &[f32], y: &[f32], target: f32) -> Option<StepMetrics> {
    if t.len() != y.len() || t.len() < 10 || target == 0.0 {
        return None;
    }
    let sign = target.signum();
    let t10 = t
        .iter()
        .zip(y)
        .find(|(_, &y)| y * sign >= 0.1 * target * sign)
        .map(|(&t, _)| t)?;
    let t90 = t
        .iter()
        .zip(y)
        .find(|(_, &y)| y * sign >= 0.9 * target * sign)
        .map(|(&t, _)| t)?;

    let peak = y.iter().fold(0.0f32, |m, &v| m.max(v * sign));
    let overshoot = ((peak - target * sign) / target.abs()).max(0.0);

    let tail = y.len() - y.len() / 5;
    let mean_tail = y[tail..].iter().sum::<f32>() / (y.len() - tail) as f32;
    let steady_state_error = (mean_tail - target).abs() / target.abs();

    Some(StepMetrics {
        rise_time: t90 - t10,
        overshoot,
        steady_state_error,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_order_response_metrics() {
        // y = 1 − e^(−t/τ), τ = 1 ms → rise time ln(9)·τ ≈ 2.197 ms.
        let tau = 1e-3;
        let (t, y): (Vec<f32>, Vec<f32>) = (0..1000)
            .map(|i| {
                let t = i as f32 * 1e-5;
                (t, 1.0 - (-t / tau).exp())
            })
            .unzip();
        let m = step_metrics(&t, &y, 1.0).unwrap();
        assert!((m.rise_time - 2.197e-3).abs() < 1e-4, "{m:?}");
        assert!(m.overshoot < 0.01, "{m:?}");
        assert!(m.steady_state_error < 0.01, "{m:?}");
    }
}
