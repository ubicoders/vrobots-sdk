//! ex39 -- rail_arm: seven joints to a pose, the vacuum on and off, and back.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex39_rail_arm          # the warehouse rail arm, sys_id 2
//! cargo run -p vrobots-examples --bin ex39_rail_arm -- <id>  # one at another id
//! ```
//!
//! A KUKA KR210 riding an 8 m linear rail, with a 400 x 300 mm vacuum pad on its
//! flange. The warehouse scene pins it at `sys_id` 2. Its base never moves --
//! the state's kinematics are the base's -- so everything this arm does is in
//! its actuator report.
//!
//! # One setpoint per joint, SI
//!
//! `SET_ARM_JOINTS` carries seven numbers in actuator order: the rail in
//! **metres**, then A1 to A6 in **radians** (`rail_arm::*` names the indices).
//! The numbers are the KUKA controller's axis values, and at home they are:
//!
//! | joint | range | top speed | home |
//! |---|---|---|---|
//! | rail | 0 to 8 m | 1.2 m/s | 4 m |
//! | A1, swivel | -185 to 185 deg | 105 deg/s | 0 |
//! | A2, shoulder | -140 to -5 deg | 94 deg/s | -60 |
//! | A3, elbow | -120 to 168 deg | 100 deg/s | 90 |
//! | A4, forearm roll | -350 to 350 deg | 136 deg/s | 0 |
//! | A5, wrist bend | -122.5 to 122.5 deg | 129 deg/s | 60 |
//! | A6, flange turn | -350 to 350 deg | 206 deg/s | 0 |
//!
//! With A4 at 0 the pad faces straight down whenever A2 + A3 + A5 = 90 deg, as at
//! home; the pose below keeps that sum. The command is a **latch**: every joint
//! is clamped to its range and driven toward its setpoint at its own top speed,
//! independently, so they arrive at different times. Nothing replies. The
//! report is the only proof, and it can end a move three ways:
//!
//! - **arrived**: every joint within a tolerance of its setpoint. The values
//!   cross the wire as `f32` both ways, so never compare with `==`.
//! - **held short**: CONTACT is not `no_contact`. Every step is tested against
//!   the scene before it is taken, and a step into anything is refused -- the
//!   protective stop -- with the setpoint still latched.
//! - **at a limit**: a setpoint past a joint's range is clamped, so that joint
//!   stops short with `joints_normalized` at -1 or 1.
//!
//! # The two entries after the joints
//!
//! | index | entry | `normalized` | `measured` |
//! |---|---|---|---|
//! | 7 | VACUUM | the latch, 0 off and 1 on | the held mass, kg; 0 = nothing |
//! | 8 | CONTACT | 0 | 0 none, 1 tool contact, 2 collision stop |
//!
//! `SET_ARM_VACUUM` is a latch too. Switched on with nothing under the pad it
//! grips nothing -- it seals only on a loose object within 1 cm of the pad,
//! square to it -- so up in the air the proof is the latch echoing back with a
//! held mass of 0. The table arm reports nine entries too, laid out differently
//! after the joints: `rail_arm_actuators()` decodes by robot type, never by
//! count (ex40).
//!
//! # Leaving it as found
//!
//! The joints are read at the start and driven back there at the end, and the
//! vacuum latch is put back as it was. An arm found holding a load is left
//! alone: switching the vacuum off would drop it. `reset()` would also put the
//! arm home -- instantly, with its setpoints -- but home is not necessarily where
//! it was found. Scene-authored: attached by `sys_id`, never deleted.

use vrobots_sdk::{ArmContact, RobotType, VirtualRobot, VrError, rail_arm};

const USAGE: &str = "\
The warehouse scene pins its rail arm at sys_id 2, which is the default. For one
at another id, pass it:

    cargo run -p vrobots-examples --bin ex39_rail_arm -- 2

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const SYS_ID: u32 = 2; // the warehouse scene's rail arm
const HZ: f64 = 25.0;
const MOVE_SAMPLES: u32 = 125; // 5 s; the pose below takes about 0.8
const VACUUM_SAMPLES: u32 = 10; // 0.4 s for the latch to come back
const TOLERANCE: f64 = 1e-3; // m on the rail, rad on the others

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();
    let robot = match attach(sys_id) {
        Ok(robot) => robot,
        Err(e) => {
            eprintln!(
                "no rail arm at sys_id {sys_id}: [{}] {e}\n\n{USAGE}",
                e.code()
            );
            std::process::exit(1);
        }
    };
    let s = robot.states();
    println!("attached to sys_id={} '{}'", robot.sys_id(), s.name);

    // ===== as found =====
    let found = robot.rail_arm_actuators()?;
    println!("found: {}", joints(&found.joints));
    println!(
        "       vacuum on={}, holding={} ({:.2} kg), CONTACT {}",
        found.vacuum_on,
        found.holding,
        found.held_mass_kg,
        found.contact.name()
    );
    if found.holding {
        println!(
            "the pad is holding {:.2} kg, and this example switches the vacuum off: nothing \
             was sent. Release it first, or reset() the arm.",
            found.held_mass_kg
        );
        return Ok(());
    }

    // ===== to a pose =====
    // Home is (4 m, 0, -60, 90, 0, 60, 0). This pose slides the carriage a metre,
    // swings A1 30 deg and takes the shoulder 10 deg back, and A5 follows so that
    // A2 + A3 + A5 stays 90 deg: the pad keeps facing straight down.
    let pose = [5.0, deg(30.0), deg(-70.0), deg(90.0), 0.0, deg(70.0), 0.0];
    println!("\n-- to {} --", joints(&pose));
    move_to(&robot, &pose)?;

    // ===== the vacuum, on and off =====
    vacuum(&robot, true)?;
    vacuum(&robot, false)?;

    // ===== hand it back =====
    println!("\n-- back to where it was found --");
    move_to(&robot, &found.joints)?;
    robot.set_arm_vacuum(found.vacuum_on)?;
    println!(
        "joints back, vacuum on={}. Scene-authored: left as it was found, never deleted.",
        found.vacuum_on
    );
    Ok(())
}

/// Attach to `sys_id`, and make sure it really is the rail arm.
///
/// `connect()` takes the robot type on trust, and the table arm reports nine
/// actuator entries too. `z/properties` names each one; once this handle has
/// read it (ex44), `rail_arm_actuators()` refuses a robot whose layout is not
/// the rail's linear joint, six servos, VACUUM and CONTACT.
fn attach(sys_id: u32) -> Result<VirtualRobot, VrError> {
    let robot = VirtualRobot::connect(RobotType::RailArm, Some(sys_id))?;
    robot.properties()?;
    robot.rail_arm_actuators()?;
    Ok(robot)
}

/// Latch one setpoint per joint, and watch the measured joints converge.
///
/// The setpoint latches, so once would do; sending it every iteration costs
/// nothing and heals a command lost on the way.
fn move_to(robot: &VirtualRobot, goal: &[f64; rail_arm::JOINT_COUNT]) -> Result<(), VrError> {
    let t0 = robot.states().elapsed;
    for i in 0..MOVE_SAMPLES {
        robot.set_arm_joints(goal)?;
        robot.rate(HZ);
        let arm = robot.rail_arm_actuators()?;
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
    let arm = robot.rail_arm_actuators()?;
    println!(
        "   not there after {:.0} s. joints_normalized {:?}: a joint at -1 or 1 is clamped at \
         its limit",
        f64::from(MOVE_SAMPLES) / HZ,
        arm.joints_normalized
    );
    Ok(())
}

/// Switch the vacuum, then read the two entries after the joints.
fn vacuum(robot: &VirtualRobot, on: bool) -> Result<(), VrError> {
    println!("\n-- vacuum {} --", if on { "on" } else { "off" });
    for _ in 0..VACUUM_SAMPLES {
        robot.set_arm_vacuum(on)?;
        robot.rate(HZ);
    }
    let arm = robot.rail_arm_actuators()?;
    println!(
        "   VACUUM  [{}]: on={}, holding={}, held {:.2} kg",
        rail_arm::VACUUM,
        arm.vacuum_on,
        arm.holding,
        arm.held_mass_kg
    );
    println!(
        "   CONTACT [{}]: {} (code {})",
        rail_arm::CONTACT,
        arm.contact.name(),
        arm.contact.code()
    );
    Ok(())
}

/// The largest gap between where the joints are and where they were sent.
fn largest_error(now: &[f64], goal: &[f64]) -> f64 {
    let mut worst = 0.0_f64;
    for (a, b) in now.iter().zip(goal) {
        worst = worst.max((a - b).abs());
    }
    worst
}

/// The rail in metres, A1 to A6 in degrees.
fn joints(j: &[f64; rail_arm::JOINT_COUNT]) -> String {
    format!(
        "rail {:.3} m  A1 {:+6.1}  A2 {:+6.1}  A3 {:+6.1}  A4 {:+6.1}  A5 {:+6.1}  A6 {:+6.1} deg",
        j[rail_arm::RAIL],
        j[rail_arm::A1].to_degrees(),
        j[rail_arm::A2].to_degrees(),
        j[rail_arm::A3].to_degrees(),
        j[rail_arm::A4].to_degrees(),
        j[rail_arm::A5].to_degrees(),
        j[rail_arm::A6].to_degrees()
    )
}

/// Degrees to the radians the command takes.
fn deg(degrees: f64) -> f64 {
    degrees.to_radians()
}

/// The rail arm's sys_id: the one command-line argument, or [`SYS_ID`].
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args.next().unwrap_or_else(|| "ex39_rail_arm".to_string());
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
