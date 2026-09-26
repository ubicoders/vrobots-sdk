//! ex29 -- hello_cartpole: balance the classic underactuated problem.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex29_hello_cartpole -- <sys_id>
//! ```
//!
//! **This one takes an argument, and every example from here on does.** A
//! cart-pole is **not in the spawn catalog** -- the sandbox scene registers
//! `multirotor`, `truck` and `msd`, and asking for anything else is refused with
//! a message that names the keys that scene *does* know (the run below does it
//! once, on purpose, because a failed create is also how you enumerate a
//! catalog). So the only way in is to attach to the one the scene authored, by
//! its `sys_id` -- and **those ids are allocated at load time and keep
//! incrementing across scene loads**, so no constant in this file could stay
//! true. Find the live one with:
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex11_topic_discovery
//! ```
//!
//! # The plant
//!
//! A cart on a rail with a pole hinged on top. One actuator -- a force on the
//! cart, `SET_INVPEN`, newtons along the rail's `+x`, clamped to
//! `CartPoleConfig::max_force` -- and two things to control with it. That is what
//! "underactuated" means and why it is the textbook problem.
//!
//! It publishes in the identity `"unity"` frame, and the published kinematics is
//! the **cart's**. The pole rides the actuator channels, which are exactly three:
//!
//! | channel | meaning |
//! |---|---|
//! | `kin.lin_pos[0]` | cart position along the rail, m -- **in WORLD coordinates** |
//! | `kin.lin_vel[0]` | cart speed along the rail, m/s |
//! | `actuator.measured[0]` | the force actually applied, N (after the clamp) |
//! | `actuator.measured[1]` | pole angle theta, **radians** |
//! | `actuator.measured[2]` | pole rate theta', rad/s |
//!
//! `theta` is the signed angle from world `+y` to the hinge-to-bob direction,
//! right-handed about `+z`, wrapped into `[-pi, pi]`: **0 is balanced upright,
//! +/-pi is hanging** -- both, because that is the wrap seam, so a fallen pole
//! may print either sign. A positive `theta` leans the bob toward `-x`.
//!
//! # The rail centre is not the world origin, and it is not on the wire
//!
//! This is the one that will cost you an afternoon. The cart slides along
//! **world x** -- the simulator pins the cart's rotation to identity and freezes
//! y, z and every axis of rotation, so the rail really is the world x axis and
//! `lin_vel[0]` really is speed along it. But the rail is centred on **wherever
//! the scene parked the rig**, and the travel limits (`travel_half_range`, 4 m
//! each side) are measured from *that*, not from the world origin. Measured live:
//! this scene's cart-pole sits at `x = -14.9`.
//!
//! The simulator knows the difference internally. **It does not publish it**:
//! `actuator.measured` is those three channels and nothing else, so there is no
//! rail-relative cart position anywhere on the wire. A controller that regulates
//! `lin_pos[0]` toward zero is therefore ordering the cart 15 m to the world
//! origin, past a dead stop it cannot cross, and at `Kx = 0.5` that is a constant
//! 7 N of destabilising bias against a 20 N actuator. The pole is on the floor
//! within a second, and nothing in the printout says why.
//!
//! So **capture the origin yourself**. `reset()` teleports the cart to the pose
//! captured at its first physics step, which *is* the rail centre -- so one
//! reset, one settle, one read of `lin_pos[0]`, and every position term after
//! that is relative to a number you measured rather than one you assumed.
//!
//! # `srv/cartpole` owns the whole plant, cart mass included
//!
//! Every mass, length and limit is here, and `srv/params` (ex22) **cannot** set
//! the cart's mass: the robot re-stamps it from `cart_mass` on every parameter
//! apply, so a mass sent there is overwritten a step later. Out-of-range values
//! are not refused either, they are silently replaced by the simulator's
//! defaults -- which is why `configure_cartpole` refuses the non-positive ones
//! itself.
//!
//! One field behaves unlike anything else in the API:
//! **`initial_pole_angle_deg` is in DEGREES while the state reports radians**,
//! and setting it **re-seats the pole at rest immediately** rather than at the
//! next reset. It is the episode's initial condition, so sending it mid-swing
//! stops the swing dead. This example stands the pole up with it before the loop
//! starts, because the controller below is a *balance* loop with no swing-up in
//! it -- from the simulator's own home angle of -45 degrees it cannot catch the
//! pole at all. (Simulated offline against the linearised plant: these gains
//! recover from about 15 degrees and no more.)
//!
//! Its recovery is `reset()`, which does both halves at once -- the cart returns
//! to the rail centre *and* the pole is re-hung at rest at whatever home angle
//! was last configured.
//!
//! # The control law
//!
//! ```text
//! d = x - x_rail_centre                            NOT x
//! F = -Kth*theta - Kthd*theta' + Kx*d + Kv*x'      clamped to +/-max_force
//! ```
//!
//! The angle terms are the obvious half: drive the cart **under** the falling
//! pole. The cart terms look backwards -- a cart right of centre is pushed
//! further right -- and they are not: pushing `+x` tips the pole toward `-x`, and
//! the angle loop then chases it back through the centre. Steering a cart-pole by
//! deliberately tipping it the wrong way first is the non-minimum-phase
//! behaviour that makes this problem interesting.
//!
//! Note the asymmetry in what needs correcting: the **position** is world and
//! must have the rail centre subtracted, while the **velocity** does not -- a
//! twist is a body quantity and the cart's body is pinned to identity, so
//! `lin_vel[0]` is already along the rail. Pose is world, twist is body; the two
//! halves of `kin` do not live in the same place.
//!
//! One control cycle per **received** state sample (ex09's pacing), so `dt` is
//! the state period and not whatever the machine felt like.
//!
//! # This robot belongs to the scene
//!
//! It was not created here, so it is **never deleted** here. The force latch is
//! released on the way out; a Ctrl-C is not, and leaves the last force applied.

use std::time::Duration;

use vrobots_sdk::{CartPoleConfig, RobotType, VirtualRobot, VrError};

const USAGE: &str = "\
The cart-pole is scene-authored, and sys ids are handed out at scene load and
keep incrementing, so there is no id this file could hard-code. Pass the live
one:

    cargo run -p vrobots-examples --bin ex29_hello_cartpole -- 7

List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

// The plant, pinned so the gains below mean something.
const CART_MASS_KG: f64 = 1.0;
const POLE_LENGTH_M: f64 = 1.2;
const BOB_MASS_KG: f64 = 0.2;
const ROD_MASS_KG: f64 = 0.1;
const MAX_FORCE_N: f64 = 20.0;
const SEED_DEG: f64 = -3.0; // where the pole is stood up, DEGREES

// Balance gains. Deliberately not aggressive: at the 25 Hz state rate the
// measurement is a frame or two old, and a hot rate gain turns that delay into
// an oscillation that grows.
const K_THETA: f64 = 25.0; // N per rad
const K_THETA_DOT: f64 = 8.0; // N per rad/s
const K_X: f64 = 0.5; // N per m
const K_V: f64 = 0.8; // N per m/s

const FALLEN_RAD: f64 = 0.35; // ~20 deg: past here, re-seat rather than flail
const MAX_RESEATS: u32 = 3;
const RUN_SAMPLES: u32 = 750; // ~30 s at the 25 Hz state rate
const SAMPLE_TIMEOUT: Duration = Duration::from_millis(500);

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();

    // A failed create enumerates the catalog. (If some scene DOES know the key,
    // this spawns one -- so clean it up rather than litter.)
    match VirtualRobot::connect(RobotType::CartPole, None) {
        Err(VrError::Service(reason)) => {
            println!("create refused, and the refusal is the catalog: {reason}\n");
        }
        Ok(spawned) => {
            println!(
                "this scene DOES create cart-poles (sys_id {}); deleting it and attaching to {sys_id} as asked\n",
                spawned.sys_id()
            );
            spawned.delete()?;
        }
        // The probe is a demonstration, not a prerequisite -- a manager that does
        // not answer says nothing about the robot we are about to attach to.
        Err(other) => println!("could not probe the catalog: [{}] {other}\n", other.code()),
    }

    let robot = VirtualRobot::connect(RobotType::CartPole, Some(sys_id))?;
    println!(
        "attached to sys_id={} ({:?}), service key = {}",
        robot.sys_id(),
        robot.robot_type(),
        vrobots_sdk::topics::srv_cartpole(robot.sys_id())
    );

    // ===== pin the plant, and stand the pole up =====
    // Cart mass is owned HERE. Sending it to srv/params would be overwritten a
    // step later.
    robot.configure_cartpole(
        &CartPoleConfig::default()
            .with_cart_mass(CART_MASS_KG)
            .with_travel_half_range(4.0)
            .with_pole_rod_mass(ROD_MASS_KG)
            .with_bob_mass(BOB_MASS_KG)
            .with_pole_length(POLE_LENGTH_M)
            .with_pole_angular_damping(0.01)
            .with_max_force(MAX_FORCE_N)
            .with_initial_pole_angle_deg(SEED_DEG),
    )?;
    println!(
        "plant set; the pole is re-seated at {SEED_DEG} deg AT REST, immediately -- not at the next reset"
    );

    // ===== find the rail centre =====
    // The one number this plant needs and does not publish. reset() puts the cart
    // back at the pose captured on its first physics step, which IS the centre the
    // travel limits are measured from -- so measure it there rather than assuming
    // the world origin, which this rig is nowhere near.
    robot.reset()?;
    settle(&robot);
    let rail_centre = robot.states().kin.lin_pos[0];
    println!(
        "rail centre measured at x = {rail_centre:+.2} m (world). Every position \
         term below is relative to THAT, not to 0."
    );
    print_state(&robot, "seated", rail_centre);

    // ===== the balance loop =====
    let mut reseats = 0u32;
    let mut worst: f64 = 0.0;
    for i in 0..RUN_SAMPLES {
        // One cycle per received sample. A timeout is a status, not a fault
        // (ex19): the last force latches, so holding is the right response.
        if let Err(VrError::Timeout(_)) = robot.wait_new_state(SAMPLE_TIMEOUT) {
            println!("no new state -- holding the last force (it latches)");
            continue;
        }

        let s = robot.states();
        // Position is a WORLD quantity, so the rail centre comes off it. Velocity
        // is a BODY quantity and the cart's body is pinned to identity, so it is
        // already along the rail and needs nothing done to it.
        let x = s.kin.lin_pos[0] - rail_centre;
        let v = s.kin.lin_vel[0];
        let theta = s.actuator.measured.get(1).copied().unwrap_or(0.0);
        let theta_dot = s.actuator.measured.get(2).copied().unwrap_or(0.0);
        worst = worst.max(theta.abs());

        // Past the catch envelope this loop is not a swing-up controller, it is
        // just a cart running at a wall. Restart the episode instead: reset()
        // re-centres the cart on the rail AND re-hangs the pole at rest at the
        // home angle configured above, which is both halves in one call.
        if theta.abs() > FALLEN_RAD {
            if reseats >= MAX_RESEATS {
                println!(
                    "\nfallen past {:.0} deg {MAX_RESEATS} times -- stopping.",
                    FALLEN_RAD.to_degrees()
                );
                break;
            }
            reseats += 1;
            println!(
                "\ntheta={:+.1} deg is past the catch envelope; resetting the episode (#{reseats})",
                theta.to_degrees()
            );
            robot.set_cartpole_force(0.0)?;
            robot.reset()?;
            settle(&robot);
            continue;
        }

        let force = (-K_THETA * theta - K_THETA_DOT * theta_dot + K_X * x + K_V * v)
            .clamp(-MAX_FORCE_N, MAX_FORCE_N);
        robot.set_cartpole_force(force)?;

        if i % 25 == 0 {
            println!(
                "t={:7.2}s  theta={:+7.2} deg  theta'={:+6.2} rad/s  rail={:+6.2} m \
                 (world x={:+7.2})  x'={:+6.2} m/s  F={:+6.2} N  applied={:+6.2} N",
                s.elapsed,
                theta.to_degrees(),
                theta_dot,
                x,
                s.kin.lin_pos[0],
                v,
                force,
                s.actuator.measured.first().copied().unwrap_or(0.0)
            );
        }
    }

    // ===== hand it back =====
    // A command latches: without this the cart keeps pushing forever.
    robot.set_cartpole_force(0.0)?;
    print_state(&robot, "final ", rail_centre);
    println!(
        "worst excursion {:.2} deg, {reseats} re-seat(s). The robot belongs to the \
         scene, so it is left running -- ex29 never deletes it.",
        worst.to_degrees()
    );
    Ok(())
}

/// Let a reset or a re-seat land: services apply in phase 0 of the next physics
/// step, and the teleport needs a sample or two to reach the state stream.
fn settle(robot: &VirtualRobot) {
    for _ in 0..15 {
        robot.rate(25.0);
    }
}

/// One state line, in the units a person reads. Both cart positions, because the
/// gap between them is the whole trap.
fn print_state(robot: &VirtualRobot, label: &str, rail_centre: f64) {
    let s = robot.states();
    println!(
        "{label} t={:7.2}s  theta={:+7.2} deg  rail={:+6.2} m (world x={:+7.2})  \
         measured={:?}",
        s.elapsed,
        s.actuator
            .measured
            .get(1)
            .copied()
            .unwrap_or(0.0)
            .to_degrees(),
        s.kin.lin_pos[0] - rail_centre,
        s.kin.lin_pos[0],
        s.actuator.measured
    );
}

/// The scene-authored cart-pole's sys_id, from the one command-line argument.
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex29_hello_cartpole".to_string());
    match args.next().map(|a| a.parse::<u32>()) {
        Some(Ok(id)) => id,
        _ => {
            eprintln!("usage: {program} <sys_id>\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
