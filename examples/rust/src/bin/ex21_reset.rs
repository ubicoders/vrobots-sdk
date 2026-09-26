//! ex21 -- reset: put a robot back where it started, without restarting anything.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex21_reset -- 1   # the scene's multirotor
//! cargo run -p vrobots-examples --bin ex21_reset        # create one instead
//! ```
//!
//! `reset()` is the first of the twelve service calls this book covers, and the
//! smallest: it teleports the robot to the pose captured at its **first physics
//! step**, zeroes linear and angular velocity, rests the actuators and re-latches
//! the robot's initial command. Exactly what the simulator's own Reset button
//! does, and nothing more.
//!
//! Four things about it are worth more than the call itself.
//!
//! **A bare GET is the request.** `srv/reset` carries no payload -- the
//! simulator's vendored C# zenoh client cannot attach one, so an empty query had
//! to mean something. The practical consequence: `srv/reset` is the one service
//! key where the usual "probe it with an empty GET and see if anybody answers"
//! capability trick **performs the action**. Probe every other key freely; never
//! probe this one.
//!
//! **The ack is a receipt, not a result.** The reply is packed the instant the
//! query lands; the teleport happens in phase 0 of the next physics step, under
//! 20 ms later at 50 Hz. `Ok(())` means "the robot heard you". The state stream
//! is the confirmation, here as everywhere.
//!
//! **A live publisher wins one step later.** Commands latch, and the reset
//! re-latches the robot's *initial* command -- 1100 us idle on a multirotor. A
//! loop that keeps publishing 1700 climbs straight back out of the reset and
//! barely notices it happened, which is exactly what you want from a controller
//! under test. This example therefore **stops commanding** across the reset, so
//! the effect is visible: watch `echo` fall from 1700 to 1100 with nothing sent.
//!
//! **Time does not restart.** `seq` and `elapsed` keep advancing straight through
//! -- only the robot moves. A frozen `elapsed` means the simulator stopped
//! (ex19), never that it reset.
//!
//! # "Home" is not "where you found it"
//!
//! Home is the pose captured at the robot's **first physics step**. Attach to a
//! scene robot that has been flying since the scene loaded and the two are
//! nothing like each other -- measured live, an attach found one **27 m** from
//! its home, so a program that treated the attach position as home reported the
//! reset as having moved the robot *away* from where it belonged.
//!
//! There is no service that reads the home pose out, so the only honest way to
//! learn it is to go there: **reset once, settle, and read the position off the
//! state stream** -- which is exactly what this example does on the attach path,
//! and exactly the trick ex29 uses to find a cart-pole's rail centre. On the
//! create path it is unnecessary: nothing has happened to the robot yet, so the
//! first sample already is home.
//!
//! What survives a reset: everything the *other* services set -- mass, inertia,
//! noise models, rotor curves, skin, coordinate frames. It is a state reset, not
//! a factory reset. What does not survive: on a fixed wing the control mode goes
//! back to onboard and the estimate source back to truth (ex31, ex33).
//!
//! # Which multirotor, and one simulator bug
//!
//! With no argument this **creates** its own multirotor and deletes it again, so
//! that running it twice disturbs nothing (ex04's lifecycle). Give it a `sys_id`
//! and it attaches to the scene's own instead and leaves it running -- resetting
//! a scene robot is benign, it is the same thing the sim's Reset button does.
//!
//! Prefer the argument for now. **As of sim v3.0.0 a client-created multirotor
//! spawns with a rigidbody that never integrates**: it hangs where it spawned,
//! ignores every pulse width and even a direct body force, while its actuator
//! echo and rotor-speed model answer perfectly normally. Trucks and MSDs created
//! the same way have live physics, and the scene's own multirotor flies. So on a
//! created robot the climb below goes nowhere and the "the latch is still flying
//! it" phase is only a story -- the reset assertions still hold, because a
//! teleport is a teleport, but you will not see it fly.
//!
//! The multirotor publishes in `"frd"`, so `lin_pos[2]` is DOWN and altitude is
//! its negation.

use vrobots_sdk::{RobotType, State, VirtualRobot, VrError};

const USAGE: &str = "\
With no argument this CREATES a multirotor and deletes it again. With a sys_id it
attaches to the scene's own and never deletes it:

    cargo run -p vrobots-examples --bin ex21_reset -- 1

Prefer the argument until the created-multirotor physics bug is fixed (see the
file header). List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const CLIMB_US: f64 = 1700.0; // well over hover: it leaves in a hurry
const HZ: f64 = 25.0; // the state rate -- one line per sample's worth of time
const CLIMB_SAMPLES: u32 = 75; // ~3 s under power
const COAST_SAMPLES: u32 = 25; // ~1 s with nothing sent: the latch keeps flying it
const SETTLE_SAMPLES: u32 = 50; // ~2 s watching it come home

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let target = target_from_args();
    let robot = VirtualRobot::connect(RobotType::Multirotor, target)?;
    let sys_id = robot.sys_id();

    println!(
        "{} sys_id={sys_id}, service key = {}",
        if target.is_none() {
            "created"
        } else {
            "attached to"
        },
        vrobots_sdk::topics::srv_reset(sys_id)
    );

    // ===== phase 0: where IS home? =====
    let home = learn_home(&robot, target.is_none())?;
    println!("home  {}\n", line(&home));

    // ===== phase 1: leave home =====
    println!("-- climbing at {CLIMB_US} us --");
    for i in 0..CLIMB_SAMPLES {
        robot.set_mr_pwm([CLIMB_US; 4])?;
        if i % 25 == 0 {
            println!("fly   {}", line(&robot.states()));
        }
        robot.rate(HZ);
    }

    // Nothing is sent from here on. The last command LATCHES, so the robot keeps
    // climbing -- a command is a setpoint, not an impulse. (On a CREATED
    // multirotor it never left the ground in the first place; see the header.)
    println!("\n-- nothing sent: the 1700 us latch is still flying it --");
    for i in 0..COAST_SAMPLES {
        if i % 12 == 0 {
            println!("coast {}", line(&robot.states()));
        }
        robot.rate(HZ);
    }

    // ===== phase 2: reset =====
    let before = robot.states();
    println!("\n-- reset() (a bare GET) --");
    robot.reset()?;
    println!(
        "acked. That is a RECEIPT: the teleport lands in phase 0 of the next \
         physics step, and the state stream is the proof."
    );

    for i in 0..SETTLE_SAMPLES {
        let s = robot.states();
        if i < 4 || i % 12 == 0 {
            println!(
                "home? {}  d(home)={:5.2} m",
                line(&s),
                distance(s.kin.lin_pos, home.kin.lin_pos)
            );
        }
        robot.rate(HZ);
    }

    // ===== what actually happened =====
    let after = robot.states();
    println!(
        "\nposition:   {:.2} m from home before, {:.2} m after",
        distance(before.kin.lin_pos, home.kin.lin_pos),
        distance(after.kin.lin_pos, home.kin.lin_pos)
    );
    println!(
        "actuators:  echo {:?} -> {:?}  (nothing was sent -- reset re-latched the \
         robot's INITIAL command)",
        before.actuator.pwm, after.actuator.pwm
    );
    println!(
        "time:       seq {} -> {}, elapsed {:.2}s -> {:.2}s  (the clock never \
         resets; only the robot moved)",
        before.seq, after.seq, before.elapsed, after.elapsed
    );

    if target.is_none() {
        robot.delete()?;
        println!("deleted sys_id={sys_id}");
    } else {
        println!("sys_id={sys_id} belongs to the scene: left running, never deleted.");
    }
    Ok(())
}

/// Learn the pose `reset()` returns this robot to.
///
/// "Home" is the pose captured at the robot's **first physics step**, and that is
/// emphatically **not** "wherever you found it". On a robot this program just
/// created the two coincide, because nothing has happened to it yet -- the first
/// sample `connect()` waited for *is* the home pose.
///
/// On a **scene** robot they can be tens of metres apart: it has been flying
/// since the scene loaded, and whatever it drifted to before you attached says
/// nothing about where it started. Measured live: an attach found one 27 m from
/// its home, and a program that assumed otherwise reported the reset as having
/// moved the robot *away*.
///
/// So on the attach path, go there and look: reset once, let the teleport land,
/// and read the position off the state stream. The same trick ex29 uses to find
/// a cart-pole's rail centre -- when the simulator will not tell you a reference,
/// put the robot on it and measure.
fn learn_home(robot: &VirtualRobot, created: bool) -> Result<State, VrError> {
    if created {
        return Ok(robot.states());
    }

    println!("attached: resetting once to find out where home actually is");
    robot.reset()?;
    for _ in 0..SETTLE_SAMPLES {
        robot.rate(HZ);
    }
    Ok(robot.states())
}

/// One state line: the fields a reset is visible in.
fn line(s: &State) -> String {
    let [x, y, z] = s.kin.lin_pos;
    let [vx, vy, vz] = s.kin.lin_vel;
    let speed = (vx * vx + vy * vy + vz * vz).sqrt();
    format!(
        "seq={:<6} t={:6.2}s pos=({x:+7.2},{y:+7.2},{z:+7.2}) [{}] alt={:5.2} m \
         |v|={speed:5.2} m/s echo={:?}",
        s.seq,
        s.elapsed,
        s.coord_frame_id,
        -z, // "frd": the third component is DOWN
        s.actuator.pwm
    )
}

/// Straight-line distance between two positions, in whatever frame they share.
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The robot to reset: a scene-authored `sys_id` if one was given, otherwise
/// `None`, which creates a fresh one. See [`USAGE`].
fn target_from_args() -> Option<u32> {
    let mut args = std::env::args();
    let program = args.next().unwrap_or_else(|| "ex21_reset".to_string());
    let arg = args.next()?;
    match arg.parse::<u32>() {
        Ok(sys_id) => Some(sys_id),
        Err(_) => {
            eprintln!("usage: {program} [sys_id]\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
