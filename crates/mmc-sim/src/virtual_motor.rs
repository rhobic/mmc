//! The virtual motor rig behind the `mmc-hal` traits: PMSM model + ideal
//! inverter + (for now, ideal) sensors. Drop-in replacement for real hardware
//! from the control loop's point of view.

use mmc_core::transforms::{clarke, Abc, AlphaBeta};
use mmc_hal::{BusVoltageSense, CurrentSense, PositionSensor, PwmOutput};

use crate::motor::{PmsmModel, PmsmParams};

pub struct VirtualMotor {
    pub motor: PmsmModel,
    pub v_bus: f32,
    /// External load torque [N·m].
    pub load_torque: f32,
    /// Physics substep [s]; must stay well under L/R.
    pub physics_dt: f32,
    duties: [f32; 3],
    enabled: bool,
    time: f64,
}

impl VirtualMotor {
    pub fn new(params: PmsmParams, v_bus: f32) -> Self {
        Self {
            motor: PmsmModel::new(params),
            v_bus,
            load_torque: 0.0,
            physics_dt: 1e-6,
            duties: [0.0; 3],
            enabled: false,
            time: 0.0,
        }
    }

    /// Simulation time [s].
    pub fn time(&self) -> f64 {
        self.time
    }

    /// Stator-frame terminal voltage produced by the latched duties.
    /// Star-connected balanced load: v_x = (d_x − mean(d)) · v_bus.
    fn v_ab(&self) -> AlphaBeta {
        if !self.enabled {
            // Approximation: a disabled (high-Z) stage is modeled as zero
            // volts. Good enough until the inverter model grows diodes.
            return AlphaBeta::default();
        }
        let mean = (self.duties[0] + self.duties[1] + self.duties[2]) / 3.0;
        clarke(Abc {
            a: (self.duties[0] - mean) * self.v_bus,
            b: (self.duties[1] - mean) * self.v_bus,
            c: (self.duties[2] - mean) * self.v_bus,
        })
    }

    /// Advance the physics by one control period with the latched duties
    /// (average-value inverter: switching ripple is not modeled).
    pub fn advance(&mut self, dt: f32) {
        let n = ((dt / self.physics_dt).round() as usize).max(1);
        let sub = dt / n as f32;
        let v_ab = self.v_ab();
        for _ in 0..n {
            self.motor.step(v_ab, self.load_torque, sub);
        }
        self.time += dt as f64;
    }
}

impl PwmOutput for VirtualMotor {
    fn set_duties(&mut self, duties: [f32; 3]) {
        self.duties = duties.map(|d| d.clamp(0.0, 1.0));
    }

    fn enable(&mut self) {
        self.enabled = true;
    }

    fn disable(&mut self) {
        self.enabled = false;
    }
}

impl CurrentSense for VirtualMotor {
    fn phase_currents(&mut self) -> [f32; 3] {
        let i = self.motor.phase_currents();
        [i.a, i.b, i.c]
    }
}

impl BusVoltageSense for VirtualMotor {
    fn vbus(&mut self) -> f32 {
        self.v_bus
    }
}

impl PositionSensor for VirtualMotor {
    fn mechanical_angle(&mut self) -> f32 {
        self.motor.theta_m
    }
}
