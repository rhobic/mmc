//! `FixHfi` built for a Cortex-M0+ (thumbv6m, no FPU), with motor 3's
//! parameters, for `tools/m0_cycles.py` to run in an instruction emulator
//! and count cycles. Two entry points, no runtime: the emulator calls
//! `fixq_bench_init` once, then `fixq_bench_tick` per control period.

#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use mmc_core::fixq::{FixHfi, FixParams};

static mut DRIVE: MaybeUninit<FixHfi> = MaybeUninit::uninit();
static mut TARGET: i32 = 0;
/// The last tick's duties (Q15), for the emulator's motor model.
#[no_mangle]
pub static mut FIXQ_DUTIES: [i32; 3] = [0; 3];

fn params() -> FixParams {
    FixParams {
        i_base: 3.0,
        v_base: 32.0,
        dt: 1e-4,
        r: 1.4177,
        l: 0.000356,
        flux: 0.006645,
        cur_bw: 1000.0,
        speed_kp: 0.004324,
        speed_ki: 0.06487,
        iq_limit: 1.2,
        omega_accel: 300.0,
        hfi_v: 1.0,
        hfi_xi: 0.055,
        hfi_bw: 150.0,
        hfi_xsat: 0.44,
        id_inject: 0.5,
        pol_a: 0.6,
        pol_s: 0.006,
        pol_n: 8,
        lock_s: 0.3,
        lock_ramp_s: 0.15,
        stuck_s: 0.5,
        advance_periods: 1.0,
        omega_max: 700.0,
    }
}

/// Setup (soft float; not timed): build the drive, target 20 rad/s el.
#[no_mangle]
pub extern "C" fn fixq_bench_init() {
    let p = params();
    unsafe {
        (*core::ptr::addr_of_mut!(DRIVE)).write(FixHfi::new(&p));
        TARGET = FixHfi::target_units(&p, 20.0);
    }
}

/// One control period: Q15 currents and bus voltage in, i_q command out
/// (duties in `FIXQ_DUTIES`).
#[no_mangle]
pub extern "C" fn fixq_bench_tick(ia: i32, ib: i32, ic: i32, vbus: i32) -> i32 {
    unsafe {
        let d = (*core::ptr::addr_of_mut!(DRIVE)).assume_init_mut();
        let o = d.step([ia, ib, ic], vbus, TARGET);
        FIXQ_DUTIES = o.duties;
        o.iq_cmd
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
