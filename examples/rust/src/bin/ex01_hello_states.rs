//! ex01 -- hello_states: read the robot's state at your own rate.
//!
//! Run with the sim in Play mode:
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex01_hello_states
//! ```
//!
//! Note the shape: `main` does setup, then owns a plain infinite loop. There is no
//! base class, no runner and no `update()` callback -- the SDK never calls your
//! code. `states()` is always the latest snapshot, so there is no "did new data
//! arrive?" flag to check either.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**;
//! `ex11_topic_discovery` lists what is actually publishing.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const HZ: f64 = 50.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        let s = robot.states(); // immutable latest snapshot, never torn
        let [x, y, z] = s.kin.lin_pos;
        println!("State t={:.3} pos=({x:.3},{y:.2},{z:.2})", s.elapsed);
        robot.rate(HZ); // drift-compensated pacing, Hz
    }
}
