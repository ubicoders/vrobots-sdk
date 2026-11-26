//! ex40 -- table_arm: six joints, a two-finger gripper in metres, and what the fingers report.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex40_table_arm          # the table-arm scene's UR10, sys_id 0
//! cargo run -p vrobots-examples --bin ex40_table_arm -- <id>  # one at another id
//! ```
//!
//! A UR10 with a Robotiq 2F-85 gripper, at the back of the table-arm scene's
//! sorting cell, pinned at `sys_id` 0. The base never moves.
//!
//! # Joints
//!
//! `SET_ARM_JOINTS` takes six setpoints, A1 to A6 -- base, shoulder, elbow and
//! three wrists -- in **radians**, each within -360 to 360 deg, reached at
//! 120 deg/s (A1-A2) or 180 deg/s (A3-A6). It is the rail arm's command (ex39)
//! with one joint fewer, and the same rules: a latch, every joint clamped and
//! driven independently, every step tested against the scene first. This arm
//! also refuses to drive into **itself**. Home is whatever pose the arm loaded
//! in, and nothing on the wire publishes it -- so read the joints once after
//! connecting, which is also how this example knows where to come back to.
//!
//! # The gripper is in metres
//!
//! `SET_ARM_GRIPPER` takes the gap between the pads in **metres**, clamped to
//! 0 ..= 0.085 (`table_arm::GRIPPER_STROKE_M`, the 2F-85's stroke). The clamp
//! is silent, so a millimetre figure such as `40.0` opens the gripper fully.
//! The fingers move at 0.15 m/s of opening, the whole stroke in under 0.6 s.
//!
//! Closing on an object stops the pads at its width, and the object is **held**
//! when both pads press it from inside and it is a loose object within the
//! 5 kg payload. Closed on thin air, as here, the pads simply arrive and nothing
//! is held.
//!
//! # The three entries after the joints
//!
//! | index | entry | `normalized` | `measured` |
//! |---|---|---|---|
//! | 6 | GRIPPER | the latch, as a fraction of the stroke | the gap between the pads now, m |
//! | 7 | CONTACT | 0 | 0 none, 1 tool contact, 2 collision stop |
//! | 8 | HOLD | 1 holding, 0 not | the held mass, kg |
//!
//! Nine entries, like the rail arm's, and laid out differently after the
//! joints: `table_arm_actuators()` decodes them by robot type, never by count.
//! A finger stopping on an object is **not** a contact -- that is how the
//! gripper feels it -- so CONTACT speaks only for the arm's joint steps.
//!
//! # Leaving it as found
//!
//! The joints and the gripper latch are read at the start and put back at the
//! end. An arm found holding something is left alone: opening would drop it.
//! Scene-authored: attached by `sys_id`, never deleted.

use vrobots_sdk::{ArmContact, RobotType, TableArmActuators, VirtualRobot, VrError, table_arm};

const USAGE: &str = "\
The table-arm scene pins its UR10 at sys_id 0, which is the default. For one at
another id, pass it:

    cargo run -p vrobots-examples --bin ex40_table_arm -- 0

The scene has to be loaded and in Play mode; its scene card needs the Pro tier.
List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const SYS_ID: u32 = 0; // the table-arm scene's UR10
const A1_TURN_DEG: f64 = 15.0; // swing the base
const A6_TURN_DEG: f64 = 45.0; // and turn the gripper about its own axis
const GRIP_M: f64 = 0.040; // METRES, so 40 mm. A bare 40.0 would open fully
const HZ: f64 = 25.0;
const MOVE_SAMPLES: u32 = 125; // 5 s; this move takes well under 1
const GRIP_SAMPLES: u32 = 125; // 5 s: the whole stroke takes 0.6, or 4.3 at srv/arm's slowest
const TOLERANCE: f64 = 1e-3; // rad
const GRIP_TOLERANCE_M: f64 = 0.0005; // half a millimetre

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();
    let robot = match attach(sys_id) {
        Ok(robot) => robot,
        Err(e) => {
            eprintln!(
                "no table arm at sys_id {sys_id}: [{}] {e}\n\n{USAGE}",
                e.code()
            );
            std::process::exit(1);
        }
    };
    let s = robot.states();
    println!("attached to sys_id={} '{}'", robot.sys_id(), s.name);

    // ===== as found =====
    // There is no home pose on the wire: these joints are the only record of it.
    let found = robot.table_arm_actuators()?;
    println!("found: {}", joints(&found.joints));
    println!("       {}", tool(&found));
    if found.holding {
        println!(
            "the gripper is holding {:.2} kg, and this example opens it: nothing was sent. \
             Let go of it first, or reset() the arm.",
            found.held_mass_kg
        );
        return Ok(());
    }

    // ===== joints =====
    let mut goal = found.joints;
    goal[table_arm::A1] += deg(A1_TURN_DEG);
    goal[table_arm::A6] += deg(A6_TURN_DEG);
    println!(
        "\n-- A1 {A1_TURN_DEG:+} deg, A6 {A6_TURN_DEG:+} deg: to {} --",
        joints(&goal)
    );
    move_to(&robot, &goal)?;

    // ===== close the gripper to a width, then open it =====
    gripper(&robot, GRIP_M)?;
    gripper(&robot, found.gripper_command_m)?;

    // ===== hand it back =====
    println!("\n-- back to where it was found --");
    move_to(&robot, &found.joints)?;
    println!("joints and gripper back. Scene-authored: left as it was found, never deleted.");
    Ok(())
}

/// Attach to `sys_id`, and make sure it really is the table arm.
///
/// `connect()` takes the robot type on trust, and the rail arm reports nine
/// actuator entries too. `z/properties` names each one; once this handle has
/// read it (ex44), `table_arm_actuators()` refuses a robot whose layout is not
/// six servos, GRIPPER, CONTACT and HOLD.
fn attach(sys_id: u32) -> Result<VirtualRobot, VrError> {
    let robot = VirtualRobot::connect(RobotType::TableArm, Some(sys_id))?;
    robot.properties()?;
    robot.table_arm_actuators()?;
    Ok(robot)
}

/// Latch six setpoints and watch the measured joints converge -- or stop short,
/// with CONTACT saying why.
fn move_to(robot: &VirtualRobot, goal: &[f64; table_arm::JOINT_COUNT]) -> Result<(), VrError> {
    let t0 = robot.states().elapsed;
    for i in 0..MOVE_SAMPLES {
        // A latch: once would do, and every iteration heals a lost command.
        robot.set_arm_joints(goal)?;
        robot.rate(HZ);
        let arm = robot.table_arm_actuators()?;
        let t = robot.states().elapsed - t0;
        let worst = largest_error(&arm.joints, goal);
        if worst < TOLERANCE {
            println!("   t={t:5.2}s  {}  -- arrived", joints(&arm.joints));
            return Ok(());
        }
        if arm.contact != ArmContact::NoContact {
            println!(
                "   t={t:5.2}s  {}  -- held short: CONTACT {} (code {})",
                joints(&arm.joints),
                arm.contact.name(),
                arm.contact.code()
            );
            return Ok(());
        }
        if i % 5 == 0 {
            println!(
                "   t={t:5.2}s  {}  worst error {worst:.4}",
                joints(&arm.joints)
            );
        }
    }
    let arm = robot.table_arm_actuators()?;
    println!(
        "   not there after {:.0} s. joints_normalized {:?}: a joint at -1 or 1 is clamped at \
         its limit",
        f64::from(MOVE_SAMPLES) / HZ,
        arm.joints_normalized
    );
    Ok(())
}

/// Command an opening, metres, and read GRIPPER, HOLD and CONTACT once the pads
/// have arrived -- or once they have closed on something.
fn gripper(robot: &VirtualRobot, opening_m: f64) -> Result<(), VrError> {
    println!("\n-- gripper to {:.1} mm --", opening_m * 1e3);
    let t0 = robot.states().elapsed;
    for _ in 0..GRIP_SAMPLES {
        robot.set_arm_gripper(opening_m)?;
        robot.rate(HZ);
        let arm = robot.table_arm_actuators()?;
        if arm.holding || (arm.gripper_opening_m - opening_m).abs() < GRIP_TOLERANCE_M {
            println!("   t={:5.2}s  {}", robot.states().elapsed - t0, tool(&arm));
            return Ok(());
        }
    }
    // Stopped short with nothing held: something is between the pads that the
    // gripper will not hold -- fixed scenery, or over the payload.
    println!(
        "   stopped short, holding nothing: {}",
        tool(&robot.table_arm_actuators()?)
    );
    Ok(())
}

/// The three entries after the joints, decoded.
fn tool(arm: &TableArmActuators) -> String {
    format!(
        "GRIPPER [{}]: latch {:.1} mm, pads {:.1} mm apart   HOLD [{}]: holding={}, {:.2} kg   \
         CONTACT [{}]: {} (code {})",
        table_arm::GRIPPER,
        arm.gripper_command_m * 1e3,
        arm.gripper_opening_m * 1e3,
        table_arm::HOLD,
        arm.holding,
        arm.held_mass_kg,
        table_arm::CONTACT,
        arm.contact.name(),
        arm.contact.code()
    )
}

/// The largest gap between where the joints are and where they were sent.
fn largest_error(now: &[f64], goal: &[f64]) -> f64 {
    let mut worst = 0.0_f64;
    for (a, b) in now.iter().zip(goal) {
        worst = worst.max((a - b).abs());
    }
    worst
}

/// A1 to A6, in degrees.
fn joints(j: &[f64; table_arm::JOINT_COUNT]) -> String {
    format!(
        "A1 {:+7.1}  A2 {:+7.1}  A3 {:+7.1}  A4 {:+7.1}  A5 {:+7.1}  A6 {:+7.1} deg",
        j[table_arm::A1].to_degrees(),
        j[table_arm::A2].to_degrees(),
        j[table_arm::A3].to_degrees(),
        j[table_arm::A4].to_degrees(),
        j[table_arm::A5].to_degrees(),
        j[table_arm::A6].to_degrees()
    )
}

/// Degrees to the radians the command takes.
fn deg(degrees: f64) -> f64 {
    degrees.to_radians()
}

/// The table arm's sys_id: the one command-line argument, or [`SYS_ID`].
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args.next().unwrap_or_else(|| "ex40_table_arm".to_string());
    let Some(arg) = args.next() else {
        return SYS_ID;
    };
    match arg.parse::<u32>() {
        Ok(sys_id) => sys_id,
        Err(_) => {
            eprintln!("usage: {program} [sys_id]\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
