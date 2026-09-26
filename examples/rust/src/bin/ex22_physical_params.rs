//! ex22 -- physical_params: change the mass under a running controller.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex22_physical_params -- 1   # the scene's multirotor
//! cargo run -p vrobots-examples --bin ex22_physical_params        # create one instead
//! ```
//!
//! `srv/params` carries two numbers, mass and the principal moments of inertia,
//! and it is the **only** channel for either. That matters more than it sounds:
//!
//! > **Neither figure appears in the state message.** Mass properties are
//! > quasi-static configuration, not state, so there is nothing to read back and
//! > nothing to diff. The confirmation is entirely **behavioural** -- the same
//! > pulse width has to produce a different acceleration, or the change did not
//! > land.
//!
//! So this example flies a fixed 1600 us collective twice, once at 1.0 kg and
//! once at 2.5 kg, and compares the climb rate. Same command, same rotors, same
//! air: a heavier aircraft climbs slower, and that is the whole receipt.
//!
//! Which is also the reason the service exists. It works **mid-flight**, so
//! changing the mass under a running loop is the standard way to test a
//! controller against a payload it was not tuned for.
//!
//! # Two silent failures the SDK refuses on your behalf
//!
//! The simulator does not validate these; it keeps the prefab's value and acks
//! `ok`, so a bad request looks exactly like a good one from out here:
//!
//! | you send | the sim does | what you would see |
//! |---|---|---|
//! | `mass <= 0` | keeps the body's current mass | nothing |
//! | a moment of inertia that is not strictly positive on **all three** axes | keeps Unity's collider-derived tensor | nothing |
//!
//! [`set_physical_params`] therefore refuses both before anything is published,
//! as `VrError::InvalidArgument` naming the field. A half-filled inertia triple
//! -- two axes set, one left at zero -- is the classic way to think you changed
//! the inertia and not have; the third block below shows it being caught.
//!
//! # Frames
//!
//! Moments of inertia are read in **your** header frame (the default here is
//! `"unity"`) and permuted into the robot's. They are positive quantities, so
//! unlike a force or a rate the conversion never flips a sign -- it only reorders
//! the triple.
//!
//! # Not the cart-pole
//!
//! A cart-pole re-stamps its cart's mass from `srv/cartpole` on every parameter
//! apply, so a mass sent here is overwritten a step later on that one robot type.
//! Use `configure_cartpole` there (ex29).
//!
//! # Which multirotor, and why attaching costs you the undo
//!
//! With no argument this **creates** a multirotor and deletes it again. Give it a
//! `sys_id` and it attaches to the scene's own -- and then **never deletes it**,
//! which has a consequence worth stating plainly:
//!
//! > The mass and inertia set below **stay set**, for that robot, for every other
//! > client, until the scene is reloaded. `reset()` does not undo them (ex21: it
//! > is a state reset, not a factory reset), and there is **no getter** -- neither
//! > figure is in the state message, so the SDK cannot read the old value first
//! > and put it back. Write down what you started with, or reload the scene.
//!
//! Prefer the argument for now anyway. **As of sim v3.0.0 a client-created
//! multirotor spawns with a rigidbody that never integrates**: it hangs where it
//! spawned and ignores every pulse width and even a direct body force, while its
//! actuator echo answers normally. Both climb runs then read 0.00 m/s and the
//! comparison shows nothing -- not because the service failed, but because
//! nothing in that robot moves. Trucks and MSDs created the same way have live
//! physics; the scene's multirotor flies.
//!
//! [`set_physical_params`]: vrobots_sdk::VirtualRobot::set_physical_params

use vrobots_sdk::{PhysicalParams, RobotType, VirtualRobot, VrError};

const USAGE: &str = "\
With no argument this CREATES a multirotor and deletes it again. With a sys_id it
attaches to the scene's own, never deletes it, and LEAVES THE MASS CHANGED until
the scene is reloaded:

    cargo run -p vrobots-examples --bin ex22_physical_params -- 1

Prefer the argument until the created-multirotor physics bug is fixed (see the
file header) -- on a created one both climb runs read 0.00 m/s. List what is
publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

// High enough that BOTH masses still climb -- a run that ends up sitting on the
// ground measures nothing, and "0.00 m/s" would not distinguish a heavy aircraft
// from a request that never landed.
const COLLECTIVE_US: f64 = 1800.0;
const LIGHT_KG: f64 = 1.0;
const HEAVY_KG: f64 = 2.0;
const MOI: [f64; 3] = [0.02, 0.02, 0.04]; // kg.m^2, in OUR header frame
const HZ: f64 = 25.0;
const SETTLE_SAMPLES: u32 = 25; // ~1 s for the reset and the new mass to bite
const MEASURE_SAMPLES: u32 = 50; // ~2 s of climb per run

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let target = target_from_args();
    let robot = VirtualRobot::connect(RobotType::Multirotor, target)?;
    println!(
        "{} sys_id={}, service key = {}",
        if target.is_none() {
            "created"
        } else {
            "attached to"
        },
        robot.sys_id(),
        vrobots_sdk::topics::srv_params(robot.sys_id())
    );
    if target.is_some() {
        println!(
            "NOTE: this robot belongs to the scene. The mass and inertia set below \
             stay set until the scene is reloaded -- there is no getter to restore \
             them from.\n"
        );
    } else {
        println!();
    }

    // ===== run 1: light =====
    robot.set_physical_params(&PhysicalParams::default().with_mass(LIGHT_KG).with_moi(MOI))?;
    let light = climb_run(&robot, LIGHT_KG)?;

    // ===== run 2: heavy, same command =====
    robot.set_physical_params(&PhysicalParams::default().with_mass(HEAVY_KG))?;
    let heavy = climb_run(&robot, HEAVY_KG)?;

    println!(
        "\n{COLLECTIVE_US} us on every rotor, {:.1} s of climb, twice:\n  \
         {LIGHT_KG} kg -> {light:+6.2} m/s\n  {HEAVY_KG} kg -> {heavy:+6.2} m/s",
        f64::from(MEASURE_SAMPLES) / HZ
    );
    println!(
        "The difference IS the receipt -- there is no mass field in the state \
         message to read back."
    );

    // ===== the two refusals =====
    println!("\n-- what the SDK refuses before anything reaches the wire --");
    show_refusal(
        "mass = 0.0",
        robot.set_physical_params(&PhysicalParams::default().with_mass(0.0)),
    );
    show_refusal(
        "moi = [0.02, 0.0, 0.04] (one axis left at zero)",
        robot.set_physical_params(&PhysicalParams::default().with_moi([0.02, 0.0, 0.04])),
    );
    show_refusal(
        "nothing set at all",
        robot.set_physical_params(&PhysicalParams::default()),
    );
    println!(
        "All three are acked `ok` by the simulator and silently ignored, which is \
         why they are caught here instead."
    );

    if target.is_none() {
        robot.delete()?;
        println!("\ndeleted sys_id={}", robot.sys_id());
    } else {
        println!(
            "\nsys_id={} belongs to the scene: left running at {HEAVY_KG} kg, and it \
             stays there until the scene is reloaded.",
            robot.sys_id()
        );
    }
    Ok(())
}

/// The multirotor to fly: a scene-authored `sys_id` if one was given, otherwise
/// `None`, which creates a fresh one. See [`USAGE`].
fn target_from_args() -> Option<u32> {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex22_physical_params".to_string());
    let arg = args.next()?;
    match arg.parse::<u32>() {
        Ok(sys_id) => Some(sys_id),
        Err(_) => {
            eprintln!("usage: {program} [sys_id]\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}

/// Reset to home, hold a fixed collective, and return the mean climb rate (m/s).
///
/// Height comes from `kin.lin_pos[2]`, negated because the multirotor publishes
/// in `"frd"` and the third component is DOWN.
///
/// **Not from `env.agl`**, which looks like the obvious field and is a trap: as
/// of sim v3.0.0 it is a hard-coded zero for *every* robot. The simulator would
/// need a downward raycast to fill it and deliberately publishes 0 rather than
/// guessing, on the grounds that `env` is the truth block and an invented height
/// is worse than an absent one. It is a placeholder, not a measurement.
fn climb_run(robot: &VirtualRobot, mass_kg: f64) -> Result<f64, VrError> {
    println!("-- {mass_kg} kg --");

    // Start each run from the same place, so the two are comparable. The reset
    // does not undo the mass: configuration survives a state reset (ex21).
    robot.reset()?;
    for _ in 0..SETTLE_SAMPLES {
        robot.set_mr_pwm([COLLECTIVE_US; 4])?;
        robot.rate(HZ);
    }

    let start = robot.states();
    for i in 0..MEASURE_SAMPLES {
        robot.set_mr_pwm([COLLECTIVE_US; 4])?;
        if i % 25 == 0 {
            let s = robot.states();
            println!(
                "   t={:6.2}s alt={:7.2} m  climb={:+6.2} m/s  echo={:?}",
                s.elapsed,
                -s.kin.lin_pos[2], // "frd": the third component is DOWN
                -s.kin.lin_vel[2],
                s.actuator.pwm
            );
        }
        robot.rate(HZ);
    }
    let end = robot.states();

    let seconds = end.elapsed - start.elapsed;
    Ok(if seconds > 0.0 {
        // "frd": z counts DOWN, so a climb is a DECREASE. The summary and the
        // live `climb=` column above must agree, or one of them is lying.
        (start.kin.lin_pos[2] - end.kin.lin_pos[2]) / seconds
    } else {
        0.0
    })
}

/// Print a refusal the way a caller should read one: code, then the sim-side
/// behaviour it is standing in for.
fn show_refusal(what: &str, result: Result<(), VrError>) {
    match result {
        Ok(()) => println!("  {what:<48} UNEXPECTED: accepted"),
        Err(e) => println!("  {what:<48} [{}] {}", e.code(), e.detail()),
    }
}
