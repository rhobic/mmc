# Motor 2 — profiler vs. Trinamic QBL5704 datasheet

Profiler captures in this directory vs. `hw/qbl5704_datasheet_v1.05.pdf`
(QMot QBL5704 family, V1.05 2020-AUG-14). The family has two variants —
QBL5704-**94**-04-032 and QBL5704-**116**-04-042 — identical except for
length/winding; both are 4-pole, 3-phase, 36 V, 4000 RPM. Which one is on the
bench is easiest to check physically: motor body length 94 mm vs 116 mm.
The L comparison below leans toward the **-116-04-042**.

## Side by side

| Quantity | Profiler (this rig) | Datasheet -94 | Datasheet -116 | Agreement |
|---|---|---|---|---|
| Pole pairs | 2 (set; kt cross-check below) | 4 poles = 2 pp | 4 poles = 2 pp | exact |
| kt | 56.2 mN·m/A (sine, 1.5·pp·ψ, peak phase A) | 63 mN·m/A (block) | 63 mN·m/A | −11% (convention + tol.) |
| ψ (flux linkage) | 18.74 ± 0.37 mWb | — (via kt) | — | consistent |
| R | 1.047 Ω *drive path* | 0.225 Ω winding (0.45 Ω line-line /2) | 0.175 Ω winding | see attribution |
| L (per phase) | 0.377 mH | 0.70 mH (1.4 line-line /2) | 0.50 mH | −116 within ±30% tol; −94 unlikely |
| Lq/Ld (saliency) | 1.07, \|ξ\|=0.036±0.004 | not published | not published | (surface-PM family) |
| Rotor inertia J | **25.1 µN·m·s²** (accel fit, 2026-07-19) | 17.3 µkg·m² | 23 µkg·m² | −116 + ~9% coupling |
| Friction (150–350 el) | 14.8 mN·m | — | — | bearing + windage |
| Rated speed | ψ predicts ≈3700 RPM at 36 V (see below) | 4000 RPM @36 V | 4000 RPM @36 V | ~7% |

## R: the 1.047 Ω is the *drive path*, and it closes numerically

The probe measures resistance as the control loop sees it — winding **plus**
everything in series on this board. From the schematics/datasheets in `hw/`:

- STSPIN830 conducting switch: ≈ 0.5 Ω (datasheet: R_DSon HS+LS = 1 Ω typ,
  i.e. ~0.5 Ω per switch, one conducting per leg at a time)
- 0.33 Ω low-side shunt, duty-weighted ≈ ×0.97 at probe duties → ≈ 0.32 Ω
- winding: 0.175 Ω (-116) or 0.225 Ω (-94)

Sum: **0.99 Ω (-116) / 1.05 Ω (-94)** vs measured **1.047 Ω** — the gap
between "datasheet R" and "profiler R" is fully accounted for by the inverter.
This is the right number for control (current PI, observer, R̂ monitor all see
the path), but it is ~5–6× the winding-only figure — do not compare it to a
datasheet directly. On motor 1 the same ~0.8 Ω path resistance dominates its
0.904 Ω reading, implying that motor's winding is ≈0.1 Ω.

## kt / ψ cross-checks (both confirm pole pairs = 2)

- kt_sine = 1.5·pp·ψ = 1.5·2·18.74 mWb = **56.2 mN·m/A** vs datasheet
  **63 mN·m/A** (block-commutation figure): −11%, within the block-vs-sine
  convention difference plus magnet tolerance. With pp = 4 the profiler
  figure would be 112 mN·m/A (+78%) — ruled out.
- Speed ceiling from ψ: ω ≈ 0.7·(VBUS/√3)/ψ (our no-field-weakening margin)
  → at 36 V: ≈ 777 rad/s el = 3700 RPM, right at the 4000 RPM rating;
  at the bench's ~25 V: ≈ 540 rad/s el (~2600 RPM) — sweep/accel targets
  must stay under this.

## Rig limits vs this motor (context for the captures)

- Rated phase current 5.08/6.67 A, peak 16.5/20.5 A; the IHM16M1's 1.5 A trip
  means the rig drives this motor at **≤ 25% of rated current** — torque
  ceiling ≈ 50 mN·m at 0.9 A vs 320/420 mN·m rated.
- Observed hang angle ~74–78° at 0.6–0.9 A. *Convention note (corrected
  2026-07-19):* in I-f the current rides the forced frame's q-axis, so an
  unloaded rotor hangs near 90° and load pulls the angle toward 0 — the load
  fraction is **cos γ**, not sin γ. cos 76° ≈ 0.24 ⇒ standstill-ish drag
  ≈ 8–12 mN·m, rising to the measured 14.8 mN·m at 150–350 rad/s el
  (accel-stage friction fit). From-rest I-f sync at the old fixed
  500 rad/s² el slew was drag + accel + cogging against a thin margin; the
  runtime `omega_accel` param (250 for this motor) resolved it.
- Winding thermal time constant 31/38 min — the panel's R̂/ΔT trend is
  meaningful on exactly these timescales.

## Speed-loop seeds from the datasheet (accel stage not runnable)

The accel stage could not run (firmware's fixed 150 rad/s el sensorless
handoff vs the drag above), so J was not measured. Rotor-only seeds from the
datasheet J and measured kt, at the standard 40 rad/s el design bandwidth
(`kp = J·bw/(kt·pp)`, `ki = kp·bw/4`):

| Variant | speed_kp | speed_ki |
|---|---|---|
| -94 (J=17.3 µkg·m²) | 0.0062 | 0.062 |
| -116 (J=23 µkg·m²) | 0.0082 | 0.082 |

`datasheet-derived.json` in this directory carries the -116 values +
pole_pairs for `mmc-host apply`. Any load inertia adds on top of rotor J —
scale kp accordingly. (Bench note 2026-07-18: hand-tuned kp=0.018/ki=0.036
were live on the device — kp ≈ 2.2× the rotor-only seed, consistent with
coupled load inertia — and were deliberately left in place.)
