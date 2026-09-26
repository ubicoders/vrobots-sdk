//! ex05 -- hello_car: drive the truck.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex05_hello_car
//! ```
//!
//! Same loop shape as ex02, different actuator. `SET_CAR` channels are pulse widths
//! on the 1100-2000 us band:
//!
//! | channel | 1100 | 1500 | 1900 |
//! |---|---|---|---|
//! | steer | full left | centre | full right |
//! | throttle | full reverse | stop (idle brake) | full forward |
//! | brake | released | -- | full |
//!
//! Brake is **bottom-anchored** -- 1100 is released, not 1500 -- and passing `None`
//! sends the two-channel form, which brakes nothing.
//!
//! In the test scene sys_id 0 is the truck and sys_id 1 is the multirotor;
//! `ex11_topic_discovery` lists what is actually there.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 0; // the truck in the test scene
const STEER_US: f64 = 1400.0; // left of centre
const THROTTLE_US: f64 = 1650.0; // light forward
const BRAKE_US: f64 = 1100.0; // released
const HZ: f64 = 50.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Truck, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        let s = robot.states();
        let [x, y, z] = s.kin.lin_pos;
        // lin_vel is a BODY-frame vector, so no single component is "the speed";
        // its magnitude is.
        let [vx, vy, vz] = s.kin.lin_vel;
        let speed = (vx * vx + vy * vy + vz * vz).sqrt();
        println!(
            "State t={:.3} pos=({x:.3},{y:.2},{z:.2}) speed={speed:.2} m/s echo={:?}",
            s.elapsed, s.actuator.pwm
        );

        // A gentle left arc: steering left of centre, light forward throttle.
        robot.set_car(STEER_US, THROTTLE_US, Some(BRAKE_US))?;

        robot.rate(HZ);
    }
}
