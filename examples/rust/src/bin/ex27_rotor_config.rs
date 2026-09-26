//! ex27 -- rotor_config: rebuild the airframe while it is flying.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex27_rotor_config -- 1   # the scene's multirotor
//! cargo run -p vrobots-examples --bin ex27_rotor_config        # create one instead
//! ```
//!
//! `srv/rotors` is the multirotor's own service, and it carries everything that
//! turns a pulse width into a force:
//!
//! ```text
//! thrust = (thrust_a*pwm^2 + thrust_b*pwm + thrust_c) * g            [N]
//! torque = spin_dir * (torque_a*pwm^2 + torque_b*pwm + torque_c) * g [N.m]
//! omega  = ang_vel_slope*pwm + ang_vel_intercept                     [rad/s]
//! ```
//!
//! Roll, pitch and yaw are not in that list because **there is no mixing matrix
//! anywhere in the simulator** -- moments fall out of the rotor *positions*, so
//! moving a rotor really does change the airframe's response, and an asymmetric
//! aircraft is just a different rotor list.
//!
//! # Three rules, and the third one is the trap
//!
//! **1. The list is replaced, not merged.** One verb, no upsert: the slice you
//! send becomes the whole rotor list.
//!
//! **2. So it must describe every rotor, in index order.** The count is fixed
//! when the airframe spawns and is `actuator.pwm.len()` in the state stream --
//! read it, do not assume four. A slice of the wrong length makes the simulator
//! drop the **entire** request (never a partial apply) and ack `ok` anyway. The
//! second run below does exactly that on purpose, with a curve that would make
//! the aircraft fall out of the sky, and proves it was dropped by climbing
//! anyway.
//!
//! **3. There is no read-back, and no per-field flags inside an entry.** You
//! cannot ask what the geometry currently is, and a zero is a zero coefficient
//! rather than "leave it alone" -- so `RotorSpec::default()`, the simulator's own
//! reference rotor, is the base to build on. Note what a bare `default()` means
//! for `position`: `[0, 0, 0]`, every rotor at the origin, an aircraft with
//! thrust and no control authority. Whatever you send **is** the airframe now.
//!
//! Positions are measured **from the robot's origin, not from its centre of
//! mass** -- the simulator subtracts the CoM offset itself, so a CoM-relative
//! value gets it subtracted twice. They are read in *your* header frame, which
//! here is the default `"unity"`: +x right, +y up, +z forward, so a flat rotor
//! ring lives in the x-z plane at y = 0.
//!
//! `spin_dir` is the sign of the yaw reaction torque, `+1` clockwise and `-1`
//! counter-clockwise -- and **`0` lets the simulator alternate by index** (index
//! 0 clockwise), which is how the stock airframes are built.
//!
//! # What the state stream will and will not tell you
//!
//! `actuator.measured` is rotor speed in rad/s, and it comes from
//! `ang_vel_slope`/`ang_vel_intercept` -- a **reported** line, computed from the
//! pulse width and nothing else. The third run below cuts the thrust curve to
//! 70% and leaves that line alone: the aircraft stops climbing while `measured`
//! does not move a digit. The rotor-speed echo is not evidence about thrust.
//! Height is.
//!
//! # Which multirotor, and why attaching is a one-way door
//!
//! With no argument this **creates** a multirotor and deletes it again. Give it a
//! `sys_id` and it attaches to the scene's own -- and then **never deletes it**,
//! which matters more here than anywhere else in the book:
//!
//! > This service **replaces** the rotor list and there is **no read-back**. Run
//! > it against the scene's multirotor and that aircraft is flying the ring
//! > geometry and the 70% thrust curve set below -- for every other client, until
//! > the scene is reloaded. `reset()` will not undo it (ex21: state reset, not
//! > factory reset), and the SDK cannot restore what it was never able to read.
//! > Reload the scene when you are done.
//!
//! Prefer the argument for now anyway. **As of sim v3.0.0 a client-created
//! multirotor spawns with a rigidbody that never integrates**: it hangs where it
//! spawned and ignores every pulse width and even a direct body force, while its
//! actuator echo and rotor-speed model answer perfectly normally. All three climb
//! runs then read 0.00 m/s and prove nothing. Trucks and MSDs created the same
//! way have live physics; the scene's multirotor flies.

use vrobots_sdk::{PhysicalParams, RobotType, RotorSpec, VirtualRobot, VrError};

const USAGE: &str = "\
With no argument this CREATES a multirotor and deletes it again. With a sys_id it
attaches to the scene's own, never deletes it, and LEAVES ITS ROTORS REBUILT
until the scene is reloaded -- there is no read-back to restore them from:

    cargo run -p vrobots-examples --bin ex27_rotor_config -- 1

Prefer the argument until the created-multirotor physics bug is fixed (see the
file header) -- on a created one every climb run reads 0.00 m/s. List what is
publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const MASS_KG: f64 = 1.0; // pinned, so the three runs are comparable
const ARM_M: f64 = 0.25; // hub distance from the robot origin
// High enough that the retuned airframe still climbs: a run that sits on the
// ground reads 0.00 m/s whether the thrust curve changed or the request was lost.
const COLLECTIVE_US: f64 = 1800.0;
const THRUST_SCALE: f64 = 0.70; // run 3: 70% of the reference thrust curve
const HZ: f64 = 25.0;
const SETTLE_SAMPLES: u32 = 25; // ~1 s after a reset
const MEASURE_SAMPLES: u32 = 75; // ~3 s of climb

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
        vrobots_sdk::topics::srv_rotors(robot.sys_id())
    );
    if target.is_some() {
        println!(
            "NOTE: this robot belongs to the scene. The rotor list is REPLACED below \
             and cannot be read back, so it stays rebuilt until the scene is \
             reloaded."
        );
    }
    robot.set_physical_params(&PhysicalParams::default().with_mass(MASS_KG))?;

    // The count is fixed at spawn. This is where it lives -- not in a constant.
    let rotors = robot.states().actuator.pwm.len();
    let collective = vec![COLLECTIVE_US; rotors];
    println!(
        "this airframe has {rotors} rotor(s) (actuator.pwm.len()), mass pinned at {MASS_KG} kg\n"
    );

    // ===== run 1: as spawned =====
    let stock = climb(&robot, "as spawned", &collective)?;

    // ===== run 2: the wrong number of entries =====
    // One short, and with a curve that makes almost no thrust. If the simulator
    // applied it the aircraft would drop; it does not, because a wrong-length
    // list is dropped whole -- and acked `ok` regardless.
    let short: Vec<RotorSpec> = ring(rotors.saturating_sub(1))
        .into_iter()
        .map(|r| r.with_thrust_curve(0.0, 0.0, 0.02))
        .collect();
    println!(
        "configure_rotors with {} entries for {rotors} rotors ...",
        short.len()
    );
    robot.configure_rotors(&short)?;
    println!("... returned Ok. That is a receipt, and the request was dropped:");
    let dropped = climb(&robot, "after the short list", &collective)?;

    // ===== run 3: every rotor, 70% thrust =====
    let weak: Vec<RotorSpec> = ring(rotors)
        .into_iter()
        .map(|r| {
            let d = RotorSpec::default();
            r.with_thrust_curve(
                d.thrust_a * THRUST_SCALE,
                d.thrust_b * THRUST_SCALE,
                d.thrust_c * THRUST_SCALE,
            )
        })
        .collect();
    robot.configure_rotors(&weak)?;
    let retuned = climb(&robot, "70% thrust curve", &collective)?;

    println!("\n{COLLECTIVE_US} us on every rotor, three times:");
    for run in [&stock, &dropped, &retuned] {
        println!(
            "  {:<24} climb={:+6.2} m/s   rotor speed echo={:?}",
            run.label, run.climb, run.measured
        );
    }
    println!(
        "Run 2 matches run 1: the short list never applied. Run 3 stops climbing \
         while the rotor-speed echo is unchanged -- that echo is the reported \
         ang_vel line, not a thrust measurement."
    );

    // ===== the one length the SDK does refuse =====
    match robot.configure_rotors(&[]) {
        Ok(()) => println!("\nUNEXPECTED: an empty rotor list was accepted"),
        Err(e) => println!("\nempty list -> [{}] {}", e.code(), e.detail()),
    }

    if target.is_none() {
        robot.delete()?;
        println!("deleted sys_id={}", robot.sys_id());
    } else {
        println!(
            "sys_id={} belongs to the scene: left running on the ring geometry and \
             the {THRUST_SCALE} thrust curve. Reload the scene to get its own \
             airframe back.",
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
        .unwrap_or_else(|| "ex27_rotor_config".to_string());
    let arg = args.next()?;
    match arg.parse::<u32>() {
        Ok(sys_id) => Some(sys_id),
        Err(_) => {
            eprintln!("usage: {program} [sys_id]\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}

/// `n` reference rotors laid out on a flat ring of radius [`ARM_M`].
///
/// In the default `"unity"` header frame the horizontal plane is x-z and +y is
/// up, so the ring sits at y = 0. `spin_dir` is left at 0 so the simulator
/// alternates clockwise/counter-clockwise by index, which is what keeps the yaw
/// torques cancelling.
fn ring(n: usize) -> Vec<RotorSpec> {
    (0..n)
        .map(|i| {
            let angle = std::f64::consts::FRAC_PI_4
                + (i as f64) * std::f64::consts::TAU / (n.max(1) as f64);
            RotorSpec::default().with_position([ARM_M * angle.sin(), 0.0, ARM_M * angle.cos()])
        })
        .collect()
}

/// One climb measurement.
struct Run {
    label: String,
    climb: f64,
    measured: Vec<f64>,
}

/// Reset, hold a fixed collective, and return the mean climb rate.
///
/// Height is `kin.lin_pos[2]`, negated because the multirotor publishes in
/// `"frd"` and the third component is DOWN. **Not `env.agl`**: as of sim v3.0.0
/// that field is a hard-coded zero for every robot -- filling it needs a downward
/// raycast, and the simulator publishes 0 rather than guessing, because `env` is
/// the truth block and an invented height is worse than an absent one.
///
/// `set_mr_pwm_n` takes one pulse width per rotor, however many that is --
/// `set_mr_pwm`'s fixed `[f64; 4]` would be the wrong length on anything but a
/// quad, and a wrong length is silently ignored by the robot.
fn climb(robot: &VirtualRobot, label: &str, collective: &[f64]) -> Result<Run, VrError> {
    println!("-- {label} --");
    robot.reset()?;
    for _ in 0..SETTLE_SAMPLES {
        robot.set_mr_pwm_n(collective)?;
        robot.rate(HZ);
    }

    let start = robot.states();
    for i in 0..MEASURE_SAMPLES {
        robot.set_mr_pwm_n(collective)?;
        if i % 25 == 0 {
            let s = robot.states();
            println!(
                "   t={:6.2}s alt={:7.2} m  climb={:+6.2} m/s  measured={:?}",
                s.elapsed,
                -s.kin.lin_pos[2], // "frd": the third component is DOWN
                -s.kin.lin_vel[2],
                round1(&s.actuator.measured)
            );
        }
        robot.rate(HZ);
    }
    let end = robot.states();

    let seconds = end.elapsed - start.elapsed;
    Ok(Run {
        label: label.to_string(),
        climb: if seconds > 0.0 {
            // "frd": z counts DOWN, so a climb is a DECREASE. The summary and the
            // live `climb=` column above must agree, or one of them is lying.
            (start.kin.lin_pos[2] - end.kin.lin_pos[2]) / seconds
        } else {
            0.0
        },
        measured: round1(&end.actuator.measured),
    })
}

/// One decimal, so a column of rotor speeds stays readable.
fn round1(v: &[f64]) -> Vec<f64> {
    v.iter().map(|x| (x * 10.0).round() / 10.0).collect()
}
