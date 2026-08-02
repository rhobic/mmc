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
| `qbl5704_datasheet_v1.05.pdf` | Trinamic QMot QBL5704 BLDC datasheet (motor 2) | [analog.com — QBL5704](https://www.analog.com/en/products/qbl5704.html) (Trinamic is now part of ADI) |

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
