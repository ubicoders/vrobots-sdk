//! ex41 -- hello_turret: find it, turn it by rate and by target, fire the laser, narrow its limits.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex41_hello_turret          # find the turret on the wire
//! cargo run -p vrobots-examples --bin ex41_hello_turret -- <id>  # or name it
//! ```
//!
//! A fixed pan-tilt turret: a pylon turns (**heading**, positive to the right
//! seen from above) and a head tilts (**pitch**, positive raises the barrel), with
//! a laser along the barrel and a stereo camera pair on the head. It reports in
//! CV, and its attitude **is** its joints -- but joint angles carry no axes, so
//! every number here means the same whatever the frame.
//!
//! # Finding it
//!
//! The sandbox scene authors a turret without pinning its id, so with no
//! argument this example looks for it, in two steps:
//!
//! 1. **Who is on the wire.** ex11's `list_topics`: every robot publishes a
//!    state topic, so the `vrobots/{id}/z/state` keys are the robots.
//! 2. **Which one is a turret.** Nothing in a state message names the type, and
//!    the number of actuator entries does not either -- a cart-pole reports
//!    three as well. `z/properties` does: the turret is the robot whose layout
//!    is `[servo, servo, laser]`. `read_properties` reads a candidate's with no
//!    handle to it, so nothing is attached until the turret is found, and
//!    reading changes nothing.
//!
//! # Two motion commands, one latch
//!
//! | command | carries | the turret |
//! |---|---|---|
//! | `set_turret_rates` | rad/s | turns at that rate, clamped to the max, until a limit |
//! | `set_turret_joints` | rad | turns to that target at the max rate, then holds |
//!
//! Both latch, and they share one latch: whichever arrived last is in charge. A
//! rate is the command for closing an aiming loop yourself; a target is the one
//! for "go there". Stop sending rates and the pylon keeps turning, all the way
//! to its limit -- which is why the rate phase below ends on a rate of zero.
//!
//! # The laser
//!
//! A latch of its own. While on, the simulator measures the distance from the
//! muzzle to the first thing the beam hits, up to 200 m, every physics step, and
//! that one number is what reaches the wire: `laser_range`, 0 when the laser is
//! off **or** nothing is within range. The barrel is aimed 5 deg down below so
//! that on open, flat ground the beam meets the floor about 24 m out.
//!
//! # `srv/turret`, and why the target survives it
//!
//! `configure_turret` sets the joint limits and rates (also the laser's range
//! and the stereo pair). Limits that exclude the current pose do not snap it:
//! the joint walks back to the nearest limit at its max rate. The **target stays
//! latched** -- the limit only clamps it -- so when the shipped limits come back
//! the turret walks out to its old target again. Nothing reads the limits back,
//! so "restore" means sending the shipped values; a scene that had tuned them
//! differently gets the shipped ones.
//!
//! # Leaving it as found
//!
//! The joints and the laser latch are read at the start and put back at the end,
//! the limits are restored, and the last motion command is a target -- the
//! found pose -- which the turret holds. Scene-authored: never deleted.

use std::time::Duration;

use vrobots_sdk::{
    RobotType, TurretActuators, TurretConfig, VirtualRobot, VrError, list_topics, read_properties,
    turret_actuator,
};

const USAGE: &str = "\
With no argument this example finds the turret: the sandbox scene authors one
but does not pin its id. To skip the search, pass the id:

    cargo run -p vrobots-examples --bin ex41_hello_turret -- 3

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const WINDOW: Duration = Duration::from_millis(1500); // ex11's listening window
const HZ: f64 = 25.0;
const RATE: f64 = 0.3; // rad/s of heading, to the right: ~17 deg/s
const RATE_SAMPLES: u32 = 38; // ~1.5 s of it
const AIM_HEADING_DEG: f64 = -30.0; // left of rest
const AIM_PITCH_DEG: f64 = -5.0; // barrel a little down, so the laser meets the floor
const MOVE_SAMPLES: u32 = 100; // 4 s; any move within the limits takes under 3
const LASER_SAMPLES: u32 = 10; // 0.4 s: measured every physics step, on from the next
const WATCH_SAMPLES: u32 = 25; // 1 s to watch a limit change land
const TOLERANCE: f64 = 1e-3; // rad
const NARROW_DEG: f64 = 20.0; // heading limits narrowed to +/- this
const SHIPPED_HEADING_DEG: f64 = 90.0; // the shipped heading limits, +/- this

fn main() -> Result<(), VrError> {
    // ===== setup: find the turret, or attach to the one named =====
    vrobots_sdk::init_logging("info");
    let robot = match sys_id_from_args() {
        Some(sys_id) => match attach(sys_id) {
            Ok(robot) => robot,
            Err(e) => {
                eprintln!(
                    "no turret at sys_id {sys_id}: [{}] {e}\n\n{USAGE}",
                    e.code()
                );
                std::process::exit(1);
            }
        },
        None => match find_turret() {
            Ok(Some(robot)) => robot,
            Ok(None) => {
                eprintln!("no robot on the wire has a turret's layout.\n\n{USAGE}");
                std::process::exit(1);
            }
            Err(e) => {
                eprintln!("could not look: [{}] {e}\n\n{USAGE}", e.code());
                std::process::exit(1);
            }
        },
    };
    let s = robot.states();
    println!(
        "attached to sys_id={} '{}', reporting in {:?}",
        robot.sys_id(),
        s.name,
        s.coord_frame_id
    );

    // ===== as found =====
    let found = robot.turret_actuators()?;
    println!("found: {}", report(&found));

    // ===== a rate: SET_TURRET_RATES =====
    println!(
        "\n-- heading rate {RATE} rad/s for {:.1} s --",
        f64::from(RATE_SAMPLES) / HZ
    );
    for i in 0..RATE_SAMPLES {
        robot.set_turret_rates(RATE, 0.0)?;
        robot.rate(HZ);
        if i % 10 == 9 {
            println!("   {}", report(&robot.turret_actuators()?));
        }
    }
    // A rate latches: without this the pylon would keep turning to its limit.
    robot.set_turret_rates(0.0, 0.0)?;

    // ===== a target: SET_TURRET_JOINTS =====
    let heading = deg(AIM_HEADING_DEG);
    let pitch = deg(AIM_PITCH_DEG);
    aim(&robot, heading, pitch)?;

    // ===== the laser =====
    laser(&robot, true)?;
    laser(&robot, false)?;

    // ===== srv/turret: narrow the heading limits, then restore them =====
    // Nothing is sent between the two calls but the calls themselves: the head
    // moves on the target latched above and whatever the limits now allow.
    robot.configure_turret(
        &TurretConfig::default()
            .with_heading_min(deg(-NARROW_DEG))
            .with_heading_max(deg(NARROW_DEG)),
    )?;
    println!(
        "\n-- srv/turret: heading limits +/-{NARROW_DEG} deg; the {AIM_HEADING_DEG} deg target \
         is clamped to them --"
    );
    watch(&robot)?;

    robot.configure_turret(
        &TurretConfig::default()
            .with_heading_min(deg(-SHIPPED_HEADING_DEG))
            .with_heading_max(deg(SHIPPED_HEADING_DEG)),
    )?;
    println!(
        "\n-- srv/turret: the shipped +/-{SHIPPED_HEADING_DEG} deg back. The target was latched \
         all along, so the head walks back out to it --"
    );
    watch(&robot)?;

    // ===== hand it back =====
    aim(&robot, found.heading, found.pitch)?;
    robot.set_turret_laser(found.laser_on)?;
    println!(
        "pose and laser back as found. Scene-authored: left holding that pose, never deleted."
    );
    Ok(())
}

/// Find the turret: list the robots on the wire, then ask each what it is.
///
/// Returns the attached handle, which has already read the layout -- so its
/// `turret_actuators()` is checked against it from here on -- or `None` when no
/// robot on the wire is a turret.
fn find_turret() -> Result<Option<VirtualRobot>, VrError> {
    // Step 1, ex11's: a zenoh topic is listed only if it published during the
    // window, and a robot's state publishes at 25 Hz.
    println!("listening for {WINDOW:?} for the robots on the wire ...");
    let topics = list_topics(WINDOW)?;
    let mut candidates: Vec<u32> = Vec::new();
    for t in &topics {
        let Some(sys_id) = t.sys_id else {
            continue;
        };
        if t.key == vrobots_sdk::topics::state(sys_id) && !candidates.contains(&sys_id) {
            candidates.push(sys_id);
        }
    }
    println!("robots on the wire: {candidates:?}");

    // Step 2: the layout on z/properties is what a robot says it is, and
    // reading it takes no handle.
    for sys_id in candidates {
        let p = match read_properties(sys_id) {
            Ok(p) => p,
            Err(e) => {
                println!("   sys_id {sys_id}: no layout: [{}] {e}", e.code());
                continue;
            }
        };
        let kinds: Vec<&str> = p.actuator_types.iter().map(|k| k.name()).collect();
        if p.actuator_types == turret_actuator::KINDS {
            println!("   sys_id {sys_id}: {kinds:?} -- the turret");
            return attach(sys_id).map(Some);
        }
        println!("   sys_id {sys_id}: {kinds:?} -- not a turret");
    }
    Ok(None)
}

/// Attach to `sys_id`, and make sure it really is a turret.
///
/// `connect()` takes the robot type on trust; once this handle has read
/// `z/properties` (ex44), `turret_actuators()` refuses a robot whose layout is
/// not `[servo, servo, laser]`.
fn attach(sys_id: u32) -> Result<VirtualRobot, VrError> {
    let robot = VirtualRobot::connect(RobotType::Turret, Some(sys_id))?;
    robot.properties()?;
    robot.turret_actuators()?;
    Ok(robot)
}

/// Latch a joint target and watch both joints arrive.
///
/// The target latches, so once would do; sending it every iteration costs
/// nothing and heals a command lost on the way.
fn aim(robot: &VirtualRobot, heading: f64, pitch: f64) -> Result<(), VrError> {
    println!(
        "\n-- aim: heading {:+.1} deg, pitch {:+.1} deg --",
        heading.to_degrees(),
        pitch.to_degrees()
    );
    let t0 = robot.states().elapsed;
    for i in 0..MOVE_SAMPLES {
        robot.set_turret_joints(heading, pitch)?;
        robot.rate(HZ);
        let now = robot.turret_actuators()?;
        let t = robot.states().elapsed - t0;
        if (now.heading - heading).abs() < TOLERANCE && (now.pitch - pitch).abs() < TOLERANCE {
            println!("   t={t:5.2}s  {}  -- there", report(&now));
            return Ok(());
        }
        if i % 5 == 0 {
            println!("   t={t:5.2}s  {}", report(&now));
        }
    }
    println!(
        "   not there: a target past a limit is clamped to it (srv/turret, or another client's \
         limits)"
    );
    Ok(())
}

/// Switch the laser and read the range it measures.
fn laser(robot: &VirtualRobot, on: bool) -> Result<(), VrError> {
    println!("\n-- laser {} --", if on { "on" } else { "off" });
    for _ in 0..LASER_SAMPLES {
        robot.set_turret_laser(on)?;
        robot.rate(HZ);
    }
    let now = robot.turret_actuators()?;
    println!(
        "   LASER [{}]: on={}, range {:.2} m  (0 = off, or nothing within range)",
        turret_actuator::LASER,
        now.laser_on,
        now.laser_range
    );
    Ok(())
}

/// Send nothing for a second and watch the head: whatever moves it now is the
/// target already latched, clamped by whatever limits the service just set.
fn watch(robot: &VirtualRobot) -> Result<(), VrError> {
    for i in 0..WATCH_SAMPLES {
        robot.rate(HZ);
        if i % 5 == 0 {
            println!("   {}", report(&robot.turret_actuators()?));
        }
    }
    Ok(())
}

/// The decoded report, in degrees.
fn report(t: &TurretActuators) -> String {
    format!(
        "heading {:+6.1} deg ({:+4.0}% of max rate)  pitch {:+5.1} deg ({:+4.0}%)  laser {} {:.2} m",
        t.heading.to_degrees(),
        t.heading_rate_fraction * 100.0,
        t.pitch.to_degrees(),
        t.pitch_rate_fraction * 100.0,
        if t.laser_on { "on " } else { "off" },
        t.laser_range
    )
}

/// Degrees to the radians the commands take.
fn deg(degrees: f64) -> f64 {
    degrees.to_radians()
}

/// The turret's sys_id if one was given; `None` means find it.
fn sys_id_from_args() -> Option<u32> {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex41_hello_turret".to_string());
    let arg = args.next()?;
    match arg.parse::<u32>() {
        Ok(sys_id) => Some(sys_id),
        Err(_) => {
            eprintln!("usage: {program} [sys_id]\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
