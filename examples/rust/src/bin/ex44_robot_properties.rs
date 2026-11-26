//! ex44 -- robot_properties: what a robot is, and what each actuator index means.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex44_robot_properties          # the warehouse forklift, sys_id 0
//! cargo run -p vrobots-examples --bin ex44_robot_properties -- <id>  # any robot
//! ```
//!
//! Every robot publishes `VRobotProperties` on `vrobots/{id}/z/properties`: its
//! mass, centre of gravity, inertia, and the **kind of every actuator entry**.
//! These are the values the simulation runs *with*, not per-step output, so they
//! ride their own topic -- published when the robot's wire opens, again on any
//! parameter change, and as a 1 Hz keepalive -- and `properties()` is a one-shot
//! read: subscribe, take the first sample, undeclare.
//!
//! # The layout is the type check
//!
//! `actuator.pwm`, `normalized` and `measured` in the state are index-parallel
//! arrays, and nothing in a state message says what index `i` *is* -- a wheel
//! speed in rad/s, a lift height in m, a held mass in kg. `actuator_types` does,
//! index for index, and that makes it the cheap answer to "what am I attached
//! to?": `connect()` takes the robot type on trust, while the layout is the
//! robot's own word. (ex30's capability probe, a service query timed until it
//! fails, was the only answer before this topic existed.) Once a handle has read
//! it, the typed getters -- `forklift_actuators()` and its siblings -- refuse a
//! robot whose layout contradicts theirs, which is how ex37-ex41 make sure.
//!
//! As observed on the simulator:
//!
//! | robot | `actuator_types` |
//! |---|---|
//! | forklift | wheel, servo, linear, linear |
//! | pallet robot | wheel x4, linear, servo |
//! | rail arm | linear, servo x6, vacuum, contact |
//! | table arm | servo x6, gripper, contact, hold |
//! | turret | servo, servo, laser |
//! | truck | wheel x4, servo |
//! | multirotor, half-drone | rotor per rotor |
//! | msd, cart-pole | unspecified: no kind exists for a force |
//!
//! `cg` and `inertia` are on body axes in the frame the message names, the
//! robot's own. What the properties do **not** carry is how a robot is tuned: no
//! lift speed, no wheel geometry, no joint range. No config service reads
//! anything back.
//!
//! # The new config services
//!
//! Each new robot type has one, and every one takes only the fields that are set
//! and acks a **receipt**, not a result (ex22): the state stream is the
//! confirmation. This example uses the forklift's, on the lift speed, and
//! measures the carriage to prove it landed. The others are the same shape:
//!
//! - `configure_forklift(&ForkliftConfig)` -- `srv/forklift`: drive, steering,
//!   the mast's ranges and speeds, the pulse band, the reset pulses.
//! - `configure_palletrobot(&PalletRobotConfig)` -- `srv/palletrobot`: the
//!   pulse band, the virtual mecanum geometry, the drive, the pad.
//! - `configure_arm(&ArmConfig)` -- `srv/arm`, one service for both arms: the
//!   protective stop, and the tool -- `suction_*` on the rail arm, `gripper_*`
//!   on the table arm, each arm ignoring the other's.
//! - `configure_turret(&TurretConfig)` -- `srv/turret`: joint limits and rates,
//!   the laser's range, the stereo pair (ex41).
//! - `configure_lidars(&[LidarConfigEntry])` -- `srv/lidars`, on every robot
//!   that carries one: mount, segments and layer mask, by lidar name. Pro only:
//!   on Community it refuses in its reply, one of the two services that ever
//!   say no (`srv/skin` is the other, ex23).
//!
//! None has a getter, so "restore" means sending the shipped value back, which
//! is what this example does. A setting another client had changed comes back
//! as the shipped one. Everything set survives `reset()`; nothing survives a
//! simulator restart.

use vrobots_sdk::{
    ActuatorKind, ForkliftActuators, ForkliftConfig, RobotType, VirtualRobot, VrError,
    forklift_actuator,
};

const USAGE: &str = "\
Reads any robot; the default is the warehouse scene's forklift, sys_id 0, the
only robot the srv/forklift half below runs on. For another robot, pass its id:

    cargo run -p vrobots-examples --bin ex44_robot_properties -- 1

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const SYS_ID: u32 = 0; // the warehouse scene's forklift

// The shipped lift band, for lift_m(). This example changes only the speed.
const LIFT_MIN_M: f64 = -0.215;
const LIFT_MAX_M: f64 = 1.11;

const SHIPPED_LIFT_SPEED: f64 = 0.5; // m/s, what restore sends back
const SLOW_LIFT_SPEED: f64 = 0.25; // m/s, half of it
const NEUTRAL_US: f64 = 1500.0;
const HZ: f64 = 25.0;
const MOVE_SAMPLES: u32 = 150; // 6 s; the slow move takes under 3
const MOVING_M: f64 = 0.02; // a carriage this far from its start has started

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();
    // The type is a claim: properties() below reads what the robot really is.
    let robot = match VirtualRobot::connect(RobotType::Forklift, Some(sys_id)) {
        Ok(robot) => robot,
        Err(e) => {
            eprintln!("no robot at sys_id {sys_id}: [{}] {e}\n\n{USAGE}", e.code());
            std::process::exit(1);
        }
    };

    // ===== z/properties =====
    let p = match robot.properties() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("no properties from sys_id {sys_id}: [{}] {e}", e.code());
            std::process::exit(1);
        }
    };
    let s = robot.states();
    println!(
        "sys_id {} '{}': {:.1} kg, centre of gravity ({:+.3}, {:+.3}, {:+.3}) m, in {:?}",
        robot.sys_id(),
        s.name,
        p.mass,
        p.cg[0],
        p.cg[1],
        p.cg[2],
        p.coord_frame_id
    );
    println!("inertia about the centre of gravity, kg*m^2:");
    for row in p.inertia {
        println!("   [{:12.1} {:12.1} {:12.1}]", row[0], row[1], row[2]);
    }
    if p.moment_arms.is_empty() {
        println!("moment arms: none published for this type");
    } else {
        println!("moment arms: {:?} m", p.moment_arms);
    }

    // ===== the actuator layout, index by index =====
    // Index-parallel to the state's actuator arrays: kind i is what measured[i] is.
    println!(
        "\n{} actuator entries; the state's measured value beside each:",
        p.actuator_types.len()
    );
    for (i, kind) in p.actuator_types.iter().enumerate() {
        let measured = s.actuator.measured.get(i).copied().unwrap_or(f64::NAN);
        println!(
            "   [{i}] {:<12} {measured:+10.3}  {}",
            kind.name(),
            unit(*kind)
        );
    }

    // ===== srv/forklift: only on a forklift =====
    if p.actuator_types != forklift_actuator::KINDS {
        println!(
            "\nnot a forklift's layout, so there is no srv/forklift to call here; the header \
             lists the other robots' configure_* calls. Nothing was sent."
        );
        return Ok(());
    }

    // The forks as found: the echo is the latch (report order, pwm[DRIVE] is the
    // throttle). The lift moves half the band away, whichever way there is room.
    let found_lift_us = f64::from(
        s.actuator
            .pwm
            .get(forklift_actuator::LIFT)
            .copied()
            .unwrap_or(1100),
    );
    let found_width_us = f64::from(
        s.actuator
            .pwm
            .get(forklift_actuator::FORK_WIDTH)
            .copied()
            .unwrap_or(1301),
    );
    let lift_to_us = if found_lift_us < 1500.0 {
        found_lift_us + 400.0
    } else {
        found_lift_us - 400.0
    };

    println!(
        "\n-- srv/forklift: lift_speed {SLOW_LIFT_SPEED} m/s (shipped: {SHIPPED_LIFT_SPEED}) --"
    );
    robot.configure_forklift(&ForkliftConfig::default().with_lift_speed(SLOW_LIFT_SPEED))?;
    println!("acked -- a receipt. The carriage is the proof:");
    let slow = lift_speed(&robot, lift_to_us, found_width_us)?;

    // ===== restore it =====
    println!("\n-- srv/forklift: lift_speed back to the shipped {SHIPPED_LIFT_SPEED} m/s --");
    robot.configure_forklift(&ForkliftConfig::default().with_lift_speed(SHIPPED_LIFT_SPEED))?;
    let shipped = lift_speed(&robot, found_lift_us, found_width_us)?;

    println!(
        "\nmeasured {slow:.2} m/s after asking for {SLOW_LIFT_SPEED}, {shipped:.2} m/s after \
         asking for {SHIPPED_LIFT_SPEED}. The forks are back at {found_lift_us} / \
         {found_width_us} us, the drive centred. Scene-authored: never deleted."
    );
    Ok(())
}

/// Drive the lift to `lift_us` -- drive centred, forks at `width_us` -- and
/// measure how fast the carriage travelled, m/s.
///
/// Timed over the middle of the move, from the first sample that has left the
/// start to the first that has arrived, so the command's own latency does not
/// count against the speed. Height and time come from one snapshot, decoded in
/// hand, so they belong to the same sample.
fn lift_speed(robot: &VirtualRobot, lift_us: f64, width_us: f64) -> Result<f64, VrError> {
    let want = lift_m(lift_us);
    let start = robot.forklift_actuators()?.lift_height;
    let mut left_at: Option<(f64, f64)> = None; // (elapsed, height) once it has started
    for _ in 0..MOVE_SAMPLES {
        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us)?;
        robot.rate(HZ);
        let s = robot.states();
        let Some(a) = ForkliftActuators::from_actuator(&s.actuator) else {
            continue;
        };
        match left_at {
            None => {
                if (a.lift_height - start).abs() > MOVING_M {
                    left_at = Some((s.elapsed, a.lift_height));
                }
            }
            Some((t_from, h_from)) => {
                if (a.lift_height - want).abs() < MOVING_M {
                    let metres = (a.lift_height - h_from).abs();
                    let seconds = s.elapsed - t_from;
                    println!(
                        "   {start:+.3} m -> {:+.3} m: {metres:.3} m in {seconds:.2} s mid-move = \
                         {:.2} m/s",
                        a.lift_height,
                        metres / seconds
                    );
                    // Let it cover the last couple of centimetres before the next move.
                    for _ in 0..5 {
                        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us)?;
                        robot.rate(HZ);
                    }
                    return Ok(metres / seconds);
                }
            }
        }
    }
    println!(
        "   the carriage did not arrive in {:.0} s",
        f64::from(MOVE_SAMPLES) / HZ
    );
    Ok(0.0)
}

/// Where a lift pulse puts the carriage: linear across the shipped band (ex37).
fn lift_m(us: f64) -> f64 {
    LIFT_MIN_M + (us - 1100.0) / 800.0 * (LIFT_MAX_M - LIFT_MIN_M)
}

/// What a `measured` entry of this kind is in -- the reason the layout exists.
fn unit(kind: ActuatorKind) -> &'static str {
    match kind {
        ActuatorKind::Rotor => "rad/s, rotor speed",
        ActuatorKind::Servo => "rad, a joint or steering angle",
        ActuatorKind::Wheel => "rad/s, wheel speed",
        ActuatorKind::Surface => "rad, surface deflection",
        ActuatorKind::Linear => "m, along a prismatic axis",
        ActuatorKind::Vacuum => "kg held by the suction pad, 0 = nothing",
        ActuatorKind::Laser => "m to the hit, 0 = off or none",
        ActuatorKind::Gripper => "m between the pads",
        ActuatorKind::Contact => "an ArmContact code: 0 none, 1 tool, 2 stop",
        ActuatorKind::Hold => "kg held by the tool, 0 = nothing",
        ActuatorKind::Unspecified => "no kind fits, e.g. a force in N",
        _ => "a kind newer than this SDK",
    }
}

/// The robot's sys_id: the one command-line argument, or [`SYS_ID`].
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex44_robot_properties".to_string());
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
