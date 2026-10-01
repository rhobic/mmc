"""Bench oscilloscope helper (Rigol DS1000Z over USB-TMC / VISA).

The scope sits on one motor phase terminal. Two uses so far:
- the PWM high level on a driven phase *is* V_bus — an ADC-independent check
  of the VBUS divider;
- coast-down back-EMF on the phase, for an independent flux/pole-pair read.

    python tools/scope.py arm --tb 10e-6 --level 12 [--scale 10 --offset -20]
    python tools/scope.py read [--out trace.csv]
"""

import argparse
import sys

import numpy as np
import pyvisa


def scope():
    rm = pyvisa.ResourceManager()
    res = [r for r in rm.list_resources() if r.startswith("USB") and "0x1AB1" in r]
    if not res:
        sys.exit("no Rigol scope on USB")
    s = rm.open_resource(res[0], timeout=10000)
    return s


def arm(a):
    s = scope()
    s.write(f":CHAN1:SCAL {a.scale}")
    s.write(f":CHAN1:OFFS {a.offset}")
    s.write(":CHAN1:COUP DC")
    s.write(f":TIM:MAIN:SCAL {a.tb}")
    s.write(f":TIM:MAIN:OFFS {a.delay}")
    s.write(":TRIG:MODE EDGE")
    s.write(":TRIG:EDG:SOUR CHAN1")
    s.write(f":TRIG:EDG:SLOP {a.slope}")
    s.write(f":TRIG:EDG:LEV {a.level}")
    s.write(":TRIG:SWE SING")
    s.write(":SING")  # the sweep mode alone does not arm the trigger
    import time
    time.sleep(0.5)
    print("armed:", s.query(":TRIG:STAT?").strip())


def read(a):
    s = scope()
    print("trigger:", s.query(":TRIG:STAT?").strip())
    s.write(":WAV:SOUR CHAN1")
    s.write(":WAV:MODE NORM")
    s.write(":WAV:FORM BYTE")
    pre = [float(x) for x in s.query(":WAV:PRE?").split(",")]
    raw = np.array(
        s.query_binary_values(":WAV:DATA?", datatype="B", container=np.array), dtype=float
    )
    xinc, xor, _xref, yinc, yor, yref = pre[4:10]
    v = (raw - yor - yref) * yinc
    t = xor + np.arange(len(v)) * xinc
    if a.out:
        np.savetxt(a.out, np.c_[t, v], delimiter=",", header="t,v", comments="")
    med = np.median(v)
    hi, lo = v[v > med], v[v <= med]
    print(
        f"n={len(v)} dt={xinc:.3g}s vmax={v.max():.2f} vmin={v.min():.2f} "
        f"high-median={np.median(hi):.2f} low-median={np.median(lo):.2f}"
    )
    mid = (np.median(hi) + np.median(lo)) / 2
    e = np.flatnonzero((v[:-1] < mid) & (v[1:] >= mid))
    if len(e) > 2:
        print(f"edges={len(e)} f={1 / np.median(np.diff(e)) / xinc:.1f} Hz "
              f"high-fraction={np.mean(v > mid):.3f}")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("arm")
    p.add_argument("--tb", type=float, default=10e-6, help="s/div")
    p.add_argument("--delay", type=float, default=0.0)
    p.add_argument("--level", type=float, default=12.0)
    p.add_argument("--slope", default="POS")
    p.add_argument("--scale", type=float, default=10.0, help="V/div")
    p.add_argument("--offset", type=float, default=-20.0)
    p = sub.add_parser("read")
    p.add_argument("--out")
    a = ap.parse_args()
    {"arm": arm, "read": read}[a.cmd](a)


if __name__ == "__main__":
    main()
