//! ex30 -- hello_halfdrone: two rotors, one degree of freedom, and a capability probe.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex30_hello_halfdrone -- <sys_id>
//! ```
//!
//! A half-drone is a multirotor cut in half: a bar on a pivot with a rotor at
//! each end of it, free to **roll** -- about the FRD forward axis, with the two
//! arms lying along the left-right one -- and constrained in everything else. It
//! exists so that attitude control can be taught without six degrees of freedom
//! arguing back: the plant is `I*theta'' = L2*F2 - L1*F1 + g*cos(theta)*m*(L1 -
//! L2) - c*theta'`, and the only input is the difference between two pulse
//! widths.
//!
//! Like the cart-pole it is **scene-authored**: not in the spawn catalog, so
//! attach by `sys_id`, and the ids move between sessions (ex11 lists them).
//!
//! # What makes this example worth its own file: the probe
//!
//! Every robot serves the same seven services -- `activate`, `reset`, `params`,
//! `skin`, `cameras`, `sensors`, `frames` -- and then each *type* adds its own:
//! `srv/rotors` for a multirotor, `srv/drive` for a truck, `srv/msd`,
//! `srv/cartpole`. **The half-drone adds nothing.** Two rotors and a hinge need
//! no configuration service.
//!
//! So asking it for `srv/rotors` is a GET to a key nobody serves, and that
//! answers as [`VrError::NoResponder`] after the service timeout. **That is the
//! capability probe**, and it is the only one this API has: nothing in a state
//! message names the robot's type, and a command for the wrong type is silently
//! ignored rather than refused. If you need to know what you are attached to,
//! ask it for a service only that type serves and time the answer.
//!
//! The cost is real -- a probe is a timeout, not a lookup -- so this example
//! shortens [`ConnectOptions::service_timeout`] to 3 s for the run. It is also
//! indistinguishable from a simulator that is not running, which ex11's
//! listing tells apart.
//!
//! # Exactly two pulse widths
//!
//! `SET_MR_PWM` carries one entry per rotor and the robot refuses any other
//! count with a warning you cannot see. Use
//! [`set_mr_pwm_n`](vrobots_sdk::VirtualRobot::set_mr_pwm_n) with a 2-element
//! slice; [`set_mr_pwm`](vrobots_sdk::VirtualRobot::set_mr_pwm)'s fixed `[f64;
//! 4]` is the quad's shape and is dropped here. The run below sends the
//! four-entry form first, on purpose, and shows the echo not moving -- ex08's
//! lesson with a robot that really does implement the id.
//!
//! | index | rotor | effect |
//! |---|---|---|
//! | 0 | rotor1, the FRD **left** arm | rotor1 high rolls FRD roll **positive** |
//! | 1 | rotor2, the FRD **right** arm | rotor2 high rolls it back |
//!
//! **This airframe's band tops out at 1900 us**, not the stock rotor's 2000. The
//! SDK validates against the wider 1100-2000 (it does not know the airframe), so
//! 1950 is accepted here and clamped there. A fresh spawn -- and every reset --
//! latches `[1100, 1100]`.
//!
//! # Reading the tilt
//!
//! There are no Euler angles on the state wire, so every consumer derives them
//! the same way, from `kin.quat`, ordered `[x, y, z, w]`:
//!
//! ```text
//! roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))     rad, FRD
//! rate = kin.ang_vel[0]                              rad/s, FRD roll rate
//! ```
//!
//! The bar dead-stops at its mechanical travel (70 degrees each side by
//! default), so a large enough difference just parks it against the stop.
//!
//! Scene-authored: this example never deletes the robot, and it idles both
//! rotors on the way out because a pulse width **latches**.
//!
//! [`VrError::NoResponder`]: vrobots_sdk::VrError::NoResponder
//! [`ConnectOptions::service_timeout`]: vrobots_sdk::ConnectOptions::service_timeout

use std::time::Duration;

use vrobots_sdk::{ConnectOptions, RobotType, RotorSpec, VirtualRobot, VrError};

const USAGE: &str = "\
The half-drone is scene-authored, and sys ids are handed out at scene load and
keep incrementing, so there is no id this file could hard-code. Pass the live
one:

    cargo run -p vrobots-examples --bin ex30_hello_halfdrone -- 4

List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const PROBE_TIMEOUT: Duration = Duration::from_secs(3);
const THROTTLE_US: f64 = 1600.0;
const PWM_MIN_US: f64 = 1100.0; // this airframe's floor -- and its idle
const PWM_MAX_US: f64 = 1900.0; // and its ceiling, NOT the stock rotor's 2000
const HZ: f64 = 25.0;
const HOLD_SAMPLES: u32 = 50; // ~2 s per differential setting

/// Differential pulse width added to rotor1 and subtracted from rotor2.
const SWEEP_US: [f64; 5] = [0.0, 90.0, 0.0, -90.0, 0.0];

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();

    // A capability probe is spent entirely on waiting for an answer that is not
    // coming, so budget it deliberately rather than taking the 8 s default.
    let robot = VirtualRobot::connect_with(
        RobotType::HalfDrone,
        Some(sys_id),
        ConnectOptions::default().with_service_timeout(PROBE_TIMEOUT),
    )?;
    println!(
        "attached to sys_id={} ({:?}), frame={:?}",
        robot.sys_id(),
        robot.robot_type(),
        robot.states().coord_frame_id
    );

    // ===== the probe =====
    println!("\n-- what does this robot serve? --");
    match robot.reset() {
        Ok(()) => println!("  srv/reset    answered  -> one of the seven every robot serves"),
        Err(e) => println!("  srv/reset    [{}] {}", e.code(), e.detail()),
    }
    match robot.configure_rotors(&[RotorSpec::default(); 2]) {
        Err(VrError::NoResponder(detail)) => println!(
            "  srv/rotors   NO RESPONDER after {PROBE_TIMEOUT:?} -> not a multirotor. \
             ({detail})"
        ),
        Ok(()) => println!("  srv/rotors   UNEXPECTED: something answered"),
        Err(e) => println!("  srv/rotors   [{}] {}", e.code(), e.detail()),
    }
    println!("  That timeout IS the type discovery. Nothing in a state message names the type.");

    // ===== the wrong number of pulse widths =====
    let before = robot.states().actuator.pwm.clone();
    println!("\n-- SET_MR_PWM with four entries, on a two-rotor airframe --");
    for _ in 0..HOLD_SAMPLES {
        robot.set_mr_pwm([THROTTLE_US; 4])?; // published happily; dropped there
        robot.rate(HZ);
    }
    println!(
        "  returned Ok every time; echo {:?} -> {:?}. A wrong length is refused by \
         a log line no client can see.",
        before,
        robot.states().actuator.pwm
    );

    // ===== two entries, which is the robot type, not a setting =====
    println!("\n-- SET_MR_PWM with two entries: [rotor1 (FRD left), rotor2 (FRD right)] --");
    for diff in SWEEP_US {
        let pwm = [
            (THROTTLE_US + diff).clamp(PWM_MIN_US, PWM_MAX_US),
            (THROTTLE_US - diff).clamp(PWM_MIN_US, PWM_MAX_US),
        ];
        println!("  diff={diff:+6.0} us -> pwm={pwm:?}");
        for i in 0..HOLD_SAMPLES {
            robot.set_mr_pwm_n(&pwm)?;
            if i % 25 == 0 {
                let s = robot.states();
                println!(
                    "     t={:7.2}s roll={:+7.2} deg  roll_rate={:+6.2} deg/s  \
                     echo={:?}  measured={:?}",
                    s.elapsed,
                    roll_deg(s.kin.quat),
                    s.kin.ang_vel[0].to_degrees(),
                    s.actuator.pwm,
                    s.actuator.measured
                );
            }
            robot.rate(HZ);
        }
    }

    // ===== hand it back =====
    // Every pulse width latches: without this the bar holds the last difference
    // forever. Idle is this airframe's floor, and also what a reset re-latches.
    robot.set_mr_pwm_n(&[PWM_MIN_US, PWM_MIN_US])?;
    println!(
        "\nidled at [{PWM_MIN_US}, {PWM_MIN_US}]. Scene-authored robot: left running, never deleted."
    );
    Ok(())
}

/// FRD roll in degrees from the state quaternion, ordered `[x, y, z, w]`.
///
/// There are no Euler angles on the wire, so this conversion is every consumer's
/// job -- the simulator's own panels do exactly this.
fn roll_deg(q: [f64; 4]) -> f64 {
    let [x, y, z, w] = q;
    (2.0 * (w * x + y * z))
        .atan2(1.0 - 2.0 * (x * x + y * y))
        .to_degrees()
}

/// The scene-authored half-drone's sys_id, from the one command-line argument.
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex30_hello_halfdrone".to_string());
    match args.next().map(|a| a.parse::<u32>()) {
        Some(Ok(id)) => id,
        _ => {
            eprintln!("usage: {program} <sys_id>\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
