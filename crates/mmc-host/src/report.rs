//! Dashboard generator: scans the results directory for CSV traces (plus
//! optional `.meta.json` sidecars), recomputes step metrics, and renders a
//! single self-contained HTML file. Anything — sim runs today, hardware
//! captures later — that lands a CSV in a subdirectory shows up on the next
//! `mmc-host report`.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::Serialize;

use mmc_sim::analysis::step_metrics;

/// Embedded points per run; longer traces are decimated (first/last kept).
const MAX_POINTS: usize = 1200;

const TEMPLATE: &str = include_str!("../templates/dashboard.html");

#[derive(Serialize)]
struct Dashboard {
    generated_unix: u64,
    git_rev: Option<String>,
    groups: Vec<Group>,
}

#[derive(Serialize)]
struct Group {
    name: String,
    title: String,
    /// Capture time of the group's most recent run, and the key the groups are
    /// ordered by. The dashboard is read newest-first: the question it gets
    /// asked most is "what did the last session produce", not "what is the
    /// alphabetically first milestone".
    newest_unix: Option<u64>,
    runs: Vec<Run>,
}

#[derive(Serialize)]
struct Run {
    #[serde(skip)]
    order: u64,
    name: String,
    title: String,
    description: String,
    command: Option<String>,
    unix_time: Option<u64>,
    columns: Vec<String>,
    rows: Vec<Vec<f32>>,
    total_samples: usize,
    metrics: Option<Metrics>,
    final_speed_rpm: Option<f32>,
    notes: Vec<String>,
}

#[derive(Serialize)]
struct Metrics {
    rise_time_ms: f32,
    ideal_rise_ms: Option<f32>,
    overshoot_pct: f32,
    sse_pct: f32,
}

/// Generate `out` from every CSV under `dir`. Returns the number of runs.
pub fn generate(dir: &Path, out: &Path) -> std::io::Result<usize> {
    let mut csvs = Vec::new();
    collect_csvs(dir, &mut csvs)?;
    csvs.sort();

    let mut groups: Vec<Group> = Vec::new();
    for path in &csvs {
        let run = match load_run(path) {
            Ok(run) => run,
            Err(e) => {
                eprintln!("skipping {}: {e}", path.display());
                continue;
            }
        };
        let group_name = path
            .parent()
            .and_then(|p| p.strip_prefix(dir).ok())
            .map(|p| p.to_string_lossy().replace('\\', "/"))
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| "ungrouped".to_string());
        match groups.iter_mut().find(|g| g.name == group_name) {
            Some(g) => g.runs.push(run),
            None => groups.push(Group {
                title: prettify_group(&group_name),
                name: group_name,
                newest_unix: None,
                runs: vec![run],
            }),
        }
    }
    for g in &mut groups {
        // `order` first so a group can tell a deliberate story (reference,
        // then regression, then fix); capture time breaks ties, so ad-hoc runs
        // that never set an order still read in the sequence they were taken
        // rather than alphabetically.
        g.runs.sort_by(|a, b| {
            a.order
                .cmp(&b.order)
                .then(a.unix_time.cmp(&b.unix_time))
                .then(a.name.cmp(&b.name))
        });
        g.newest_unix = g.runs.iter().filter_map(|r| r.unix_time).max();
    }
    // Newest group first; undated groups sink to the bottom.
    groups.sort_by(|a, b| b.newest_unix.cmp(&a.newest_unix).then(a.name.cmp(&b.name)));
    let n_runs = groups.iter().map(|g| g.runs.len()).sum();

    let dash = Dashboard {
        generated_unix: SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0),
        git_rev: git_rev(),
        groups,
    };
    // `<` escaped so the blob can never terminate its <script> element.
    let json = serde_json::to_string(&dash)?.replace('<', "\\u003c");
    let html = TEMPLATE.replacen("__MMC_DATA__", &json, 1);
    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(out, html)?;
    Ok(n_runs)
}

fn collect_csvs(dir: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    if !dir.exists() {
        return Ok(());
    }
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        if path.is_dir() {
            collect_csvs(&path, out)?;
        } else if path
            .extension()
            .is_some_and(|e| e.eq_ignore_ascii_case("csv"))
        {
            out.push(path);
        }
    }
    Ok(())
}

/// "ms2-current-loop" → "MS2 — current loop".
fn prettify_group(name: &str) -> String {
    let mut parts = name.splitn(2, '-');
    let first = parts.next().unwrap_or(name);
    let is_ms = first.len() > 2
        && first[..2].eq_ignore_ascii_case("ms")
        && first[2..].chars().all(|c| c.is_ascii_digit());
    match (is_ms, parts.next()) {
        (true, Some(rest)) => format!("{} — {}", first.to_uppercase(), rest.replace('-', " ")),
        (true, None) => first.to_uppercase(),
        _ => name.replace('-', " "),
    }
}

fn load_run(path: &Path) -> Result<Run, String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let mut lines = text.lines();
    let columns: Vec<String> = lines
        .next()
        .ok_or("empty file")?
        .split(',')
        .map(|s| s.trim().to_string())
        .collect();
    let mut rows: Vec<Vec<f32>> = Vec::new();
    for (i, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let row: Vec<f32> = line
            .split(',')
            .map(|v| v.trim().parse::<f32>())
            .collect::<Result<_, _>>()
            .map_err(|e| format!("row {}: {e}", i + 2))?;
        if row.len() != columns.len() {
            return Err(format!("row {}: expected {} fields", i + 2, columns.len()));
        }
        rows.push(row);
    }
    if rows.is_empty() {
        return Err("no data rows".into());
    }

    let name = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();

    let meta: serde_json::Value = std::fs::read_to_string(path.with_extension("meta.json"))
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or(serde_json::Value::Null);
    let meta_str = |key: &str| meta.get(key).and_then(|v| v.as_str()).map(String::from);

    let col = |n: &str| columns.iter().position(|c| c == n);
    let metrics = compute_step_metrics(&columns, &rows).map(|m| Metrics {
        rise_time_ms: m.rise_time * 1e3,
        ideal_rise_ms: meta
            .pointer("/params/bandwidth_rad_s")
            .and_then(|v| v.as_f64())
            .map(|bw| (9.0f32).ln() / bw as f32 * 1e3),
        overshoot_pct: m.overshoot * 100.0,
        sse_pct: m.steady_state_error * 100.0,
    });
    let locked = meta
        .pointer("/params/locked")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let final_speed_rpm = col("omega_m").filter(|_| !locked).and_then(|c| {
        rows.last()
            .map(|r| r[c] * 60.0 / (2.0 * std::f32::consts::PI))
    });

    let total_samples = rows.len();
    Ok(Run {
        order: meta.get("order").and_then(|v| v.as_u64()).unwrap_or(100),
        title: meta_str("title").unwrap_or_else(|| name.replace('_', " ")),
        description: meta_str("description").unwrap_or_default(),
        command: meta_str("command"),
        unix_time: meta.get("unix_time").and_then(|v| v.as_u64()),
        notes: meta
            .get("notes")
            .and_then(|v| v.as_array())
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect()
            })
            .unwrap_or_default(),
        name,
        rows: decimate(rows),
        total_samples,
        columns,
        metrics,
        final_speed_rpm,
    })
}

/// Step metrics recomputed from the trace itself: the step instant is where
/// `iq_ref` first changes, the target its final value.
fn compute_step_metrics(
    columns: &[String],
    rows: &[Vec<f32>],
) -> Option<mmc_sim::analysis::StepMetrics> {
    let t_col = columns.iter().position(|c| c == "t")?;
    let ref_col = columns.iter().position(|c| c == "iq_ref")?;
    let y_col = columns.iter().position(|c| c == "i_q")?;
    let step_idx = rows.iter().position(|r| r[ref_col] != rows[0][ref_col])?;
    let target = rows.last()?[ref_col];
    let t0 = rows[step_idx][t_col];
    let (t, y): (Vec<f32>, Vec<f32>) = rows[step_idx..]
        .iter()
        .map(|r| (r[t_col] - t0, r[y_col]))
        .unzip();
    step_metrics(&t, &y, target)
}

fn decimate(rows: Vec<Vec<f32>>) -> Vec<Vec<f32>> {
    if rows.len() <= MAX_POINTS {
        return rows;
    }
    let stride = rows.len().div_ceil(MAX_POINTS);
    let last = rows.len() - 1;
    rows.into_iter()
        .enumerate()
        .filter(|(i, _)| i % stride == 0 || *i == last)
        .map(|(_, r)| r)
        .collect()
}

fn git_rev() -> Option<String> {
    let out = std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn group_titles() {
        assert_eq!(prettify_group("ms2-current-loop"), "MS2 — current loop");
        assert_eq!(prettify_group("ms10-thing"), "MS10 — thing");
        assert_eq!(prettify_group("hardware-captures"), "hardware captures");
    }

    /// Groups render newest-first. Named so that alphabetical order would put
    /// the older group first — the assertion fails if recency is ignored.
    #[test]
    fn groups_render_newest_first() {
        let dir = std::env::temp_dir().join("mmc-report-order-test");
        std::fs::remove_dir_all(&dir).ok();
        let csv = "t,iq_ref,i_d,i_q\n0,1,0,0\n0.001,1,0,0.5\n0.002,1,0,0.9\n";
        for (group, unix) in [
            ("ms1-older", 1_700_000_000u64),
            ("ms2-newer", 1_800_000_000u64),
        ] {
            let sub = dir.join(group);
            std::fs::create_dir_all(&sub).unwrap();
            std::fs::write(sub.join("run.csv"), csv).unwrap();
            std::fs::write(
                sub.join("run.meta.json"),
                format!("{{\"unix_time\":{unix}}}"),
            )
            .unwrap();
        }
        let out = dir.join("index.html");
        generate(&dir, &out).unwrap();
        let html = std::fs::read_to_string(&out).unwrap();
        let newer = html.find("ms2-newer").expect("newer group present");
        let older = html.find("ms1-older").expect("older group present");
        assert!(
            newer < older,
            "newest group must come first, got {newer} vs {older}"
        );
        assert!(
            html.contains("\"newest_unix\":1800000000"),
            "recency key emitted"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn report_generates_from_csv() {
        let dir = std::env::temp_dir().join("mmc-report-test");
        let sub = dir.join("ms2-current-loop");
        std::fs::create_dir_all(&sub).unwrap();
        let mut csv = String::from("t,iq_ref,i_d,i_q\n");
        for i in 0..200 {
            let t = i as f32 * 1e-4;
            let (r, y) = if t >= 0.002 {
                (1.0, 1.0 - (-(t - 0.002) / 1e-3).exp())
            } else {
                (0.0, 0.0)
            };
            csv.push_str(&format!("{t},{r},0,{y}\n"));
        }
        std::fs::write(sub.join("step_test.csv"), csv).unwrap();
        let out = dir.join("index.html");
        let n = generate(&dir, &out).unwrap();
        assert_eq!(n, 1);
        let html = std::fs::read_to_string(&out).unwrap();
        assert!(html.contains("step_test"), "run data embedded");
        assert!(!html.contains("__MMC_DATA__"), "placeholder replaced");
        // First-order rise time ln(9)·τ ≈ 2.2 ms must be recomputed from CSV.
        assert!(html.contains("\"rise_time_ms\":2.2"), "metrics recomputed");
        std::fs::remove_dir_all(&dir).ok();
    }
}
