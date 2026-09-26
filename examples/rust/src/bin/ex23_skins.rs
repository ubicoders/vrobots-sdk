//! ex23 -- skins: the one service that ever says no.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex23_skins
//! ```
//!
//! `srv/skin` takes a catalog key and dresses the robot in it. The catalogs are
//! **per robot type**, matched case-insensitively:
//!
//! | type | keys |
//! |---|---|
//! | multirotor | `blue` `desert` `gold` `green` `mono` `pink` `snow` `white` |
//! | truck | `black` `blue` `camouflage` `gray` `red` |
//!
//! Every other robot type ships no catalog at all, so every request to one is a
//! no-op.
//!
//! # It is the only service that reports a refusal -- and only one kind
//!
//! Skins are tier-gated inside the simulator. A tier refusal comes back as an
//! honest `ok = false` **with a reason**, which the SDK surfaces as
//! `VrError::Service` carrying the simulator's own message. That is the single
//! place in this entire API surface where a service says no, and it is worth
//! knowing precisely because of what happens with everything else:
//!
//! | request | reply | what actually happened |
//! |---|---|---|
//! | a key your tier allows | `ok` | the skin changed |
//! | any key, tier too low | `ok = false` + reason | **`VrError::Service` -- do not retry, the answer will not change** |
//! | `gold` on a **truck** (a multirotor key) | `ok` | nothing; logged inside the sim |
//! | `chartreuse` (in no catalog at all) | `ok` | nothing; logged inside the sim |
//!
//! The last two rows are the lesson. A wrong *key* is not an error, it is a
//! receipt for a request that was dropped -- so a typo looks exactly like
//! success, and the confirmation is the robot in front of you. Both are
//! demonstrated below, after the five real keys.
//!
//! # On a truck a skin is not only cosmetic
//!
//! The wheel colliders travel with the skin prefab, so a swap **rebinds the
//! physics wheels**. This example keeps the truck rolling across every change so
//! that shows up on the wire: `actuator.measured[0..3]` are the four wheel speeds
//! (FL, FR, RL, RR, rad/s) and `[4]` is the steering servo. They must keep
//! turning across each swap -- a wheel channel that flatlines is a rebind that
//! did not take.
//!
//! An empty key is refused client-side: sim-side it is a payload-less read-back
//! probe, not a skin.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

/// Every key the truck catalog knows.
const TRUCK_SKINS: [&str; 5] = ["black", "blue", "camouflage", "gray", "red"];
/// A perfectly valid key -- for a multirotor. On a truck it is acked and dropped.
const WRONG_TYPE_SKIN: &str = "gold";
/// In nobody's catalog.
const UNKNOWN_SKIN: &str = "chartreuse";

const STEER_US: f64 = 1500.0; // straight ahead
const THROTTLE_US: f64 = 1600.0; // slow forward, so the wheels are always turning
const BRAKE_US: f64 = 1100.0; // released (brake is bottom-anchored, ex05)
const HZ: f64 = 25.0;
const HOLD_SAMPLES: u32 = 30; // ~1.2 s per skin

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Truck, None)?;
    println!(
        "created sys_id={}, service key = {}\n",
        robot.sys_id(),
        vrobots_sdk::topics::srv_skin(robot.sys_id())
    );

    // ===== the real catalog =====
    for skin in TRUCK_SKINS {
        if !wear(&robot, skin)? {
            // A tier refusal is final. Retrying it is the one thing this service
            // makes unambiguous, so stop rather than walk the rest of the list.
            println!("\nStopping: the tier gate does not open on a retry.");
            robot.delete()?;
            return Ok(());
        }
    }

    // ===== the two that look like success and are not =====
    println!("\n-- keys that are acked `ok` and dropped inside the simulator --");
    wear(&robot, WRONG_TYPE_SKIN)?; // a multirotor key, on a truck
    wear(&robot, UNKNOWN_SKIN)?; // no catalog has it
    println!(
        "Both returned Ok. The truck is still wearing {:?} -- the ack was a \
         receipt for a request the robot then refused with a log line no client \
         can see.",
        TRUCK_SKINS[TRUCK_SKINS.len() - 1]
    );

    // ===== and the one the SDK will not even send =====
    match robot.set_skin("   ") {
        Ok(()) => println!("\nUNEXPECTED: an empty key was accepted"),
        Err(e) => println!("\nempty key -> [{}] {}", e.code(), e.detail()),
    }

    robot.delete()?;
    println!("\ndeleted sys_id={}", robot.sys_id());
    Ok(())
}

/// Ask for one skin and keep driving through it. `Ok(false)` means the tier
/// refused -- an answer, not a failure to talk to.
fn wear(robot: &VirtualRobot, skin: &str) -> Result<bool, VrError> {
    match robot.set_skin(skin) {
        Ok(()) => println!("set_skin({skin:?}) -> ok"),
        Err(VrError::Service(reason)) => {
            // The sim's own words. This is the ONLY service that ever gets here.
            println!("set_skin({skin:?}) -> REFUSED by the sim: {reason}");
            return Ok(false);
        }
        Err(other) => return Err(other),
    }

    for i in 0..HOLD_SAMPLES {
        robot.set_car(STEER_US, THROTTLE_US, Some(BRAKE_US))?;
        if i % 15 == 0 {
            let s = robot.states();
            let [vx, vy, vz] = s.kin.lin_vel;
            println!(
                "    t={:6.2}s speed={:5.2} m/s wheels={:?} steer_servo={:?}",
                s.elapsed,
                (vx * vx + vy * vy + vz * vz).sqrt(),
                // 0..3 are FL, FR, RL, RR in rad/s; they must keep turning
                // across the swap, because the colliders were just rebound.
                &s.actuator.measured[..s.actuator.measured.len().min(4)],
                s.actuator.measured.get(4)
            );
        }
        robot.rate(HZ);
    }
    Ok(true)
}
