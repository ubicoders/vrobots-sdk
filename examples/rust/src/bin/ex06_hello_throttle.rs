//! ex06 -- hello_throttle: the *other* multirotor actuator command, and how to
//! tell a command that is ignored from one that landed.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex06_hello_throttle
//! ```
//!
//! `SET_MR_THROTTLE` is normalised per-rotor throttle: four values on 0..1
//! instead of four pulse widths. It is the command you would reach for to hover
//! without thinking in microseconds -- and it is defined on the wire but **no
//! robot type acts on it yet**, so this example is really a lesson in how that
//! looks from the client side.
//!
//! There is no reply and no error. A robot that receives an id it does not
//! implement silently ignores it, because the id space is shared across robot
//! types and "not mine" is correct behaviour, not a fault. So the only evidence
//! you ever get is the state stream, and the loop below prints all of it:
//!
//! - `actuator.pwm` -- the pulse widths the robot latched. Unchanged here.
//! - `actuator.normalized` -- the normalised command it latched, if it has one.
//! - `actuator.measured` -- what the devices actually did (rotor rad/s).
//!
//! Watch the echo across runs and you get latching for free: if you ran ex02
//! first, the pulse widths *it* latched are still there, published by a process
//! that has already exited. A command is a setpoint the robot holds, not an
//! event, and the echo reports the holder rather than the sender.
//!
//! Run ex02 side by side: identical loop, an id the sim implements, and the echo
//! moves. That contrast is the whole point of this file. Today the way to fly a
//! multirotor is `set_mr_pwm`.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const THROTTLE: [f64; 4] = [0.6, 0.6, 0.6, 0.6]; // normalised 0..1, one per rotor
const HZ: f64 = 25.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        // Published exactly like set_mr_pwm: one put on vrobots/<id>/z/cmd, no
        // reply, latched until the next one arrives.
        robot.set_mr_throttle(THROTTLE)?;

        let s = robot.states();
        // The state frame is the robot's, not yours -- "frd" here, so lin_pos[2]
        // is DOWN, and altitude above the start point is its negation.
        let [_, _, down] = s.kin.lin_pos;
        println!(
            "sent {THROTTLE:?} -> alt={:.2} m  pwm={:?} normalized={:?} measured={:?}",
            -down,
            s.actuator.pwm,
            round3(&s.actuator.normalized),
            round3(&s.actuator.measured),
        );

        robot.rate(HZ);
    }
}

/// Three decimals, so a screenful of rotor telemetry stays readable.
fn round3(values: &[f64]) -> Vec<f64> {
    values
        .iter()
        .map(|v| (v * 1000.0).round() / 1000.0)
        .collect()
}
