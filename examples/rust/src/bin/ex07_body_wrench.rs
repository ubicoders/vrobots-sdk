//! ex07 -- body_wrench: push the robot around with a force and a torque.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex07_body_wrench
//! ```
//!
//! The wrench group is the disturbance-injection channel: a wind gust, a dropped
//! payload, a contact push. Three typed wrappers cover it, and they are the
//! clearest example in the SDK of the wire's shape leaking into an API:
//!
//! | call | wire payload |
//! |---|---|
//! | `set_body_force(f)` | `vec3` = force |
//! | `set_body_torque(t)` | `vec3` = torque |
//! | `set_body_ft(f, t)` | `vec3` = force, **`vec3_arr[0]`** = torque |
//!
//! `SET_BODY_FT` is asymmetric because the schema is; the wrapper hides it, and
//! ex08 shows what filling it by hand looks like.
//!
//! **Vectors carry a frame.** Every command this SDK sends is stamped with the
//! `coord_frame_id` in the connect options -- `"unity"` by default, which is
//! left-handed X-right / Y-up / Z-forward. The robot converts your vector into
//! its own axes using the physically correct rule for the command (a force
//! converts differently from a torque, which is a pseudovector and carries the
//! handedness sign). An *untagged* vector is taken at face value and silently
//! flips sign between opposite-handed conventions, which is why the SDK always
//! tags.
//!
//! Honest caveat, the same one as ex06: **no robot type acts on these yet.** They
//! are on the wire, and silently ignored. `state.wrench` -- printed below -- is
//! the total force and torque the *simulator* has on the body, so it is where the
//! effect will appear the day the sim implements them. Until then it shows the
//! robot's own actuators and nothing of yours.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const GUST_N: [f64; 3] = [5.0, 0.0, 0.0]; // newtons, in OUR frame ("unity")
const TWIST_NM: [f64; 3] = [0.0, 0.0, 0.2]; // newton-metres, same frame
const HZ: f64 = 2.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;
    println!(
        "sending vectors tagged {:?} (axes {})",
        robot.options().coord_frame_id,
        robot.options().axis_convention.0
    );

    // ===== loop =====
    let mut step = 0u64;
    loop {
        // One verb per iteration, so each printed line names exactly what went
        // out on the wire.
        let sent = match step % 3 {
            0 => {
                robot.set_body_force(GUST_N)?;
                format!("set_body_force({GUST_N:?})")
            }
            1 => {
                robot.set_body_torque(TWIST_NM)?;
                format!("set_body_torque({TWIST_NM:?})")
            }
            _ => {
                robot.set_body_ft(GUST_N, TWIST_NM)?;
                format!("set_body_ft({GUST_N:?}, {TWIST_NM:?})")
            }
        };
        step += 1;

        let s = robot.states();
        let [fx, fy, fz] = s.wrench.force;
        let [tx, ty, tz] = s.wrench.torque;
        println!(
            "{sent}\n    state.wrench force=({fx:+.2},{fy:+.2},{fz:+.2}) N  \
             torque=({tx:+.2},{ty:+.2},{tz:+.2}) N.m  in {:?}",
            s.coord_frame_id
        );

        robot.rate(HZ);
    }
}
