//! ex04 -- hello_service: create a robot, and learn what "explicit lifecycle" means.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex04_hello_service
//! ```
//!
//! Lifecycle and configuration are one-shot request/response, so there is no loop
//! here. Three things worth internalising:
//!
//! - **Robots outlive the process.** Dropping the handle closes the session and
//!   leaves the robot flying; only `delete()` removes one. This example deletes
//!   what it created so that running it twice does not litter the scene -- comment
//!   the `delete()` out and the robot stays, which is the whole point.
//! - **Create is the one non-idempotent service.** The SDK sends it exactly once
//!   and never retries it, because every retry that reaches the manager reserves
//!   another id and spawns another robot.
//! - **The ack is a receipt, not a result.** `connect()` returns only once the new
//!   robot's state topic has published, and `delete()` returns only once it has
//!   fallen silent. Absence is the proof, both ways.
//!
//! Comment out the delete, then attach to the id it prints with `ex01_hello_states`
//! (edit `SYS_ID`) -- that is the create-then-attach lifecycle in two commands.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const ROBOT_TYPE: RobotType = RobotType::Multirotor;

fn main() -> Result<(), VrError> {
    vrobots_sdk::init_logging("info");

    // Create a NEW robot in the sim (no sys_id -> manager create; reply carries the id).
    let robot = VirtualRobot::connect(ROBOT_TYPE, None)?;
    let sys_id = robot.sys_id();
    println!("created sys_id = {sys_id}");

    // The create reply is a receipt; the robot *exists* once its state topic
    // publishes, which connect() already waited for -- so this is real data.
    let s = robot.states();
    println!(
        "first state: t={:.3} seq={} name={:?}",
        s.elapsed, s.seq, s.name
    );
    println!("its state topic: {}", vrobots_sdk::topics::state(sys_id));

    // Deletion is explicit and never implicit. delete() waits for the state topic
    // to fall silent: the manager's ack is only a receipt, absence is the proof.
    robot.delete()?;
    println!(
        "deleted sys_id = {sys_id} (is_deleted={})",
        robot.is_deleted()
    );

    // The handle is spent. Commands do not silently do nothing -- they say why.
    match robot.set_mr_pwm([1500.0; 4]) {
        Ok(()) => println!("unexpected: a deleted robot accepted a command"),
        Err(e) => println!("the handle is spent, as expected: [{}] {e}", e.code()),
    }
    Ok(())
}
