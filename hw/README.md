# Bench hardware

This is the one place the specific parts are named — everywhere else the code
and docs refer to them by function (dev board, inverter shield, gate driver,
motor 2). You need the part numbers here to obtain the right documents.

The PDFs are **not tracked in git**: they're third-party documents, freely
downloadable but not ours to redistribute, and the board schematics alone are
~20 MB. Download them into this directory as needed; `.gitignore` keeps them
untracked. The one tracked file is `g474-ihm16.ioc`, a pin-configuration export
generated for this project.

| File | What it is | Where to get it |
| --- | --- | --- |
| `mb1367-g474re-d01-schematic.pdf` | NUCLEO-G474RE board schematic (MB1367) | [st.com — NUCLEO-G474RE](https://www.st.com/en/evaluation-tools/nucleo-g474re.html) → CAD Resources |
| `mb1360-g0b1re-c02_schematic.pdf` | NUCLEO-G0B1RE board schematic (MB1360) | [st.com — NUCLEO-G0B1RE](https://www.st.com/en/evaluation-tools/nucleo-g0b1re.html) → CAD Resources |
| `x-nucleo-ihm16m1_schematic.pdf` | X-NUCLEO-IHM16M1 inverter shield schematic | [st.com — X-NUCLEO-IHM16M1](https://www.st.com/en/ecosystems/x-nucleo-ihm16m1.html) → CAD Resources |
| `stspin830.pdf` | STSPIN830 three-phase gate driver datasheet | [st.com — STSPIN830](https://www.st.com/en/motor-drivers/stspin830.html) |
| `x-nucleo-ihm07m1_schematic.pdf` | X-NUCLEO-IHM07M1 inverter shield schematic (L6230) | [st.com — X-NUCLEO-IHM07M1](https://www.st.com/en/ecosystems/x-nucleo-ihm07m1.html) → CAD Resources |
| `mb1136-default-c04_schematic.pdf` | NUCLEO-F302R8 board schematic (MB1136) | [st.com — NUCLEO-F302R8](https://www.st.com/en/evaluation-tools/nucleo-f302r8.html) → CAD Resources |
| `l6230.pdf` | L6230 three-phase DMOS driver datasheet | [st.com — L6230](https://www.st.com/en/motor-drivers/l6230.html) |
| `qbl5704_datasheet_v1.05.pdf` | Trinamic QMot QBL5704 BLDC datasheet (motor 2) | [analog.com — QBL5704](https://www.analog.com/en/products/qbl5704.html) (Trinamic is now part of ADI) |

st.com refuses scripted downloads (the request hangs with zero bytes), so
fetch these by hand in a browser.

## Benches

| Bench | MCU board | Inverter shield | Firmware crate |
| --- | --- | --- | --- |
| 1 | NUCLEO-G474RE (170 MHz M4F) | X-NUCLEO-IHM16M1 (STSPIN830) | `mmc-fw-g474` |
| 2 | NUCLEO-F302R8 (72 MHz M4F, 64 KB / 16 KB) | X-NUCLEO-IHM07M1 (L6230) | `mmc-fw-f302` |

Both shields use the same current-sense front end (0.33 Ω shunts, 680R/2.2k
bias, ×1.53) and the same 10k/2.2k BEMF dividers, which is why the drive's
`BoardSpec` numbers barely differ. Motors are not tied to a bench; check
which one is connected (the R/L probe tells them apart in seconds).

## Why each one matters here

Facts extracted from these documents are already written down in the code and
docs, so you can usually work without the PDFs:

- **IHM16M1 schematic** — the shield pin map in the `mmc-fw-g474` module header:
  TIM1 CH1-3 on PA8-10, phase enables on PB13/14/15, shunt amplifiers on
  PA1/PB1/PB0, VBUS ÷16 on PA0, CURRENT_REF on PB4. Also the BEMF divider
  network (12.2/2.2 ratio, enable on PC9) whose pin mapping was ultimately
  resolved on hardware rather than from the schematic — BEMF1=U→PC0,
  BEMF3=W→PC1, BEMF2=V→PC3, with PC2 being the SPEED potentiometer. See
  [PROGRESS.md](../docs/PROGRESS.md) session 13.
- **STSPIN830 datasheet** — R_DSon (HS+LS ≈ 1 Ω typ), which together with the
  0.33 Ω shunt explains why the profiler's R reads ~5-6× a datasheet winding
  resistance. That reconciliation is written up in
  `testresults/motor2-4pole/datasheet-comparison.md`.
- **QBL5704 datasheet** — pole count, winding R/L, and rotor inertia for motor 2;
  cross-checked against the profiler in the same comparison document.
- **Nucleo board schematics** — ST-LINK VCP routing and the EN_FAULT 0R-route
  variation (R37→PB12 on this revision, R35→PA11 on others) that the firmware
  works around by reading both pins with pull-ups.

- **IHM07M1 schematic** — the `mmc-fw-f302` pin map: L6230 IN1-3 on
  PA8-10 (TIM1), EN1-3 on PC10-12, DIAG/EN on PA6, shunt amps on
  PA0/PC1/PC0 (ADC1 IN1/IN7/IN6), VBUS on PA1 (169k/9.31k), BEMF on
  PC3/PB0/PA7 with the divider return on PC9, halls on PA15/PB3/PB10 (the
  TIM2 CH1-3 pins), CURRENT_REF on PB4. Two things it implies that bit on
  bring-up:
  - **The BEMF dividers saturate above ~18 V** (3.3 V × 12.2/2.2): on a
    30 V bus a phase driven high reads full scale. The floating phase of
    six-step swings about V_bus/2 = 15 V and still reads, but the
    driven-high reference does not.
  - **J5/J6 select single- vs three-shunt**; the firmware needs three-shunt.
- **L6230 datasheet** — R_DS(on) HS+LS ≈ 1.35 Ω typ, the board's
  `r_path` estimate (1.0 Ω with the shunt), and internal dead time: at a
  30 V bus it costs ~0.8 V of command (measured, session 30).
