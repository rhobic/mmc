"""Scope a phase terminal while the rotor coasts with the stage off.

Spins the motor up in hall FOC, cuts the drive (`--step-kind off`), and has
the Rigol (channel 1 on a phase terminal) catch the coast: a pulse-width
trigger on a positive pulse longer than the PWM can make fires on the first
back-EMF half-wave. With every switch open the lowest terminal sits a diode
drop below ground and the others carry the line voltages to it, which is
what tools/bemf_fit.py fits.

    python tools/bemf_scope.py --out testresults/motor3-bemf/coast_800.csv [--omega 800]
"""
import argparse
import math
import subprocess
import time

import numpy as np
import pyvisa


def main(a):
    rm = pyvisa.ResourceManager()
    s = rm.open_resource([r for r in rm.list_resources() if "0x1AB1" in r][0], timeout=30000)
    s.write(":RUN")
    s.write(":CHAN1:COUP DC")
    s.write(f":CHAN1:SCAL {a.vdiv}")
    s.write(f":CHAN1:OFFS {-a.vdiv * 3}")
    s.write(f":TIM:MAIN:SCAL {a.tdiv}")
    s.write(f":TIM:MAIN:OFFS {a.tdiv * 5}")  # trigger near the left edge
    s.write(":ACQ:MDEP 1200000")
    s.write(":TRIG:MODE PULS")
    s.write(":TRIG:PULS:SOUR CHAN1")
    s.write(":TRIG:PULS:WHEN PGR")
    s.write(f":TRIG:PULS:WIDT {a.min_width}")
    s.write(f":TRIG:PULS:LEV {a.level}")
    hz = a.omega / (2 * math.pi)
    cmd = [a.host, "capture", "--serial", a.serial, "--baud", "1000000", "--duration", str(a.duration),
           "--divider", "5", "--drive", "hall-foc", "--amp", "1.0", f"--hz={hz:.3f}", f"--step-hz={hz:.3f}",
           "--step-kind", "off", "--out", a.out.replace(".csv", ".telemetry.csv"),
           "--title", f"Coast from {a.omega:.0f} rad/s el (scope on a phase terminal)"]
    cap = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    # Arm only once the drive is switching: before it starts, the idle
    # terminal sits high and its first PWM edge would end a "long pulse".
    time.sleep(0.6 * a.duration - 0.6)
    s.write(":TRIG:SWE SING")
    s.write(":SING")
    print(cap.communicate()[0].strip().splitlines()[-1])
    for _ in range(50):
        if s.query(":TRIG:STAT?").strip() == "STOP":
            break
        time.sleep(0.1)
    else:
        raise SystemExit("scope never triggered")
    s.write(":WAV:SOUR CHAN1")
    s.write(":WAV:MODE RAW")
    s.write(":WAV:FORM BYTE")
    pre = [float(x) for x in s.query(":WAV:PRE?").split(",")]
    n = int(pre[2])
    chunks = []
    for k in range(1, n + 1, 250000):
        s.write(f":WAV:STAR {k}")
        s.write(f":WAV:STOP {min(k + 249999, n)}")
        chunks.append(s.query_binary_values(":WAV:DATA?", datatype="B", container=np.array))
    raw = np.concatenate(chunks).astype(float)
    xinc, xor, _, yinc, yor, yref = pre[4:10]
    v = (raw - yor - yref) * yinc
    t = xor + np.arange(len(v)) * xinc
    k = max(1, len(v) // a.points)
    m = len(v) // k
    np.savetxt(a.out, np.c_[t[:m * k].reshape(m, k).mean(1), v[:m * k].reshape(m, k).mean(1)],
               delimiter=",", header="t,v", comments="", fmt="%.7g")
    s.write(":TRIG:MODE EDGE")
    s.write(":TRIG:SWE AUTO")
    s.write(":RUN")
    print(f"scope: {len(v)} samples at {1 / xinc / 1e6:.1f} MSa/s -> {m} points in {a.out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--omega", type=float, default=800.0, help="coast-from speed [rad/s el]")
    ap.add_argument("--duration", type=float, default=7.0)
    ap.add_argument("--serial", default="COM9")
    ap.add_argument("--host", default="./target/release/mmc-host")
    ap.add_argument("--vdiv", type=float, default=2.0)
    ap.add_argument("--tdiv", type=float, default=10e-3)
    ap.add_argument("--level", type=float, default=3.0)
    ap.add_argument("--min-width", type=float, default=2e-3)
    ap.add_argument("--points", type=int, default=24000)
    main(ap.parse_args())
