//! ex02 -- hello_control: close the loop.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex02_hello_control
//! ```
//!
//! `SET_MR_PWM` is the lowest actuation level there is -- **you are the flight
//! controller**. Nothing sits between these four pulse widths and the thrust
//! curves: no attitude stabilisation, no rate damping. `[1100; 4]` is idle and a
//! flying drone falls; hover is wherever total thrust crosses weight for the
//! current mass and curves.
//!
//! Commands latch: the last one received stays in effect until the next arrives,
//! so a 25 Hz loop is fine even though physics runs at 50 Hz. There is no reply --
//! **proof that a command landed is `s.actuator.pwm` echoing it back.**
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.
//! `PWM_US` below is barely off idle and will not lift it; try 1700 to watch it
//! climb.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const PWM_US: f64 = 1501.0; // microseconds per rotor, on the 1100-2000 band
const HZ: f64 = 100.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        let s = robot.states();
        let [x, y, z] = s.kin.lin_pos;
        println!(
            "State t={:.3} pos=({x:.3},{y:.2},{z:.2}) echo={:?}",
            s.elapsed, s.actuator.pwm
        );

        // Do some COOL control here and publish -- PID/EKF is user code, NOT the SDK.
        let cool_control_result = [PWM_US; 4];
        robot.set_mr_pwm(cool_control_result)?;

        robot.rate(HZ);
    }
}
