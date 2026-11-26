//! ex37 -- hello_forklift: forks up and out, a short drive, a brake, and everything put back.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex37_hello_forklift          # the warehouse forklift, sys_id 0
//! cargo run -p vrobots-examples --bin ex37_hello_forklift -- <id>  # a forklift at another id
//! ```
//!
//! The warehouse scene pins its counterbalance forklift at `sys_id` 0, so that
//! is the default. Its front pair drives, its **rear pair steers** -- the tail
//! swings out in a turn -- and its mast carries two **position servos**. One
//! command, `SET_FORKLIFT`, drives all four, RC style:
//!
//! | channel | 1100 us | 1500 us | 1900 us |
//! |---|---|---|---|
//! | steer | full left | straight | full right (45 deg) |
//! | throttle | full reverse | stop, on the idle brake | full forward (~5 m/s) |
//! | lift | -0.215 m | 0.448 m | 1.11 m |
//! | fork width | 0.158 m | 0.746 m | 1.334 m |
//! | brake, an optional fifth | released | half | full |
//!
//! # Two kinds of channel
//!
//! Steer and throttle centre on 1500 with a +/-20 us deadband, like the truck's
//! (ex05). Lift and fork width have no centre at all: every microsecond is a
//! place on the mast, linear across the band -- the arithmetic in `lift_m()` and
//! `width_m()` below. And the mast **slews** there, the lift at 0.5 m/s and the
//! width at only 0.1 m/s, so `actuator.pwm` echoes a new pulse on the very next
//! sample while the forks are still on their way. The move is done when
//! `actuator.measured` says so, and `forklift_actuators()` decodes that into
//! metres: the typed actuator report is the confirmation, not the echo.
//!
//! # The report is not in command order
//!
//! The command is `[steer, throttle, lift, width]`. The report leads with the
//! drive -- `[drive, steering, lift, width]` -- so `actuator.pwm[0]` echoes the
//! **throttle**. `forklift_channel::*` indexes the command, `forklift_actuator::*`
//! the report.
//!
//! # The brake
//!
//! A fifth channel, sent by `set_forklift_with_brake`, and **bottom-anchored**:
//! 1100 is released and 1900 is full, not 1500. It latches with the rest of the
//! frame, and the next four-channel `set_forklift` releases it. The report has
//! no brake entry; the proof is the drive speed falling to zero.
//!
//! # Leaving it as found
//!
//! Every frame latches, so whatever this program sends last is what the
//! forklift keeps doing after it exits. The echo *is* the latch, so the forks'
//! pulses are read off it at the start and sent again at the end, with the drive
//! centred and the brake released. The forklift drives back as far as it went --
//! open loop, so to within a few centimetres. `reset()` would be quicker -- it
//! snaps the mast to the reset pulses instead of slewing it -- but it also
//! teleports the forklift to where the scene started it, which is "home", not
//! "where it was found" (ex21).
//!
//! Let it run to the end: a Ctrl-C mid-drive leaves the throttle latched, and
//! the forklift keeps driving. Scene-authored: attached by `sys_id`, never
//! deleted.

use vrobots_sdk::{ForkliftActuators, RobotType, VirtualRobot, VrError, forklift_actuator};

const USAGE: &str = "\
The warehouse scene pins its forklift at sys_id 0, which is the default. For a
forklift at another id, pass it:

    cargo run -p vrobots-examples --bin ex37_hello_forklift -- 0

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const SYS_ID: u32 = 0; // the warehouse scene's forklift

// The shipped mast ranges, for lift_m() and width_m(). srv/forklift can move
// them (ex44), and then the expectations printed below are wrong.
const LIFT_MIN_M: f64 = -0.215;
const LIFT_MAX_M: f64 = 1.11;
const WIDTH_MIN_M: f64 = 0.158;
const WIDTH_MAX_M: f64 = 1.334;
const TYRE_RADIUS_M: f64 = 0.36; // front tyres: drive rad/s times this is m/s

const NEUTRAL_US: f64 = 1500.0; // straight ahead, and stopped on the idle brake
const LIFT_UP_US: f64 = 1500.0; // the carriage to 0.448 m
const WIDTH_OUT_US: f64 = 1500.0; // the forks to 0.746 m apart
const FORWARD_US: f64 = 1600.0; // a quarter of the forward half: gentle
const BACK_US: f64 = 1400.0; // the same quarter in reverse
const BRAKE_FULL_US: f64 = 1900.0; // bottom-anchored: 1100 is released
const HZ: f64 = 20.0;
const DRIVE_SAMPLES: u32 = 50; // 2.5 s each way, about a metre
const BRAKE_SAMPLES: u32 = 40; // 2 s, far more than a stop needs
const MAST_SAMPLES: u32 = 200; // 10 s; the slow width needs ~3 s for this move
const TOLERANCE_M: f64 = 0.005;
const STOPPED_RAD_S: f64 = 0.05;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();
    let robot = match attach(sys_id) {
        Ok(robot) => robot,
        Err(e) => {
            eprintln!(
                "no forklift at sys_id {sys_id}: [{}] {e}\n\n{USAGE}",
                e.code()
            );
            std::process::exit(1);
        }
    };
    let s = robot.states();
    println!(
        "attached to sys_id={} '{}', reporting in {:?}",
        robot.sys_id(),
        s.name,
        s.coord_frame_id
    );

    // ===== as found =====
    // The echo is the latch -- the last frame anybody sent -- so it is also what
    // to send back at the end. Report order: pwm[DRIVE] is the throttle.
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
    println!(
        "found: echo {:?} = [throttle, steer, lift, width] us",
        s.actuator.pwm
    );
    println!("       {}", report(&robot.forklift_actuators()?));

    // ===== forks up, forks out =====
    println!(
        "\n-- lift {LIFT_UP_US} us, width {WIDTH_OUT_US} us: by the band {:.3} m and {:.3} m --",
        lift_m(LIFT_UP_US),
        width_m(WIDTH_OUT_US)
    );
    move_mast(&robot, LIFT_UP_US, WIDTH_OUT_US)?;

    // ===== a gentle drive forward, and the brake =====
    let start = robot.states().kin.lin_pos;
    drive(&robot, "forward", FORWARD_US, LIFT_UP_US, WIDTH_OUT_US)?;
    brake(&robot, LIFT_UP_US, WIDTH_OUT_US)?;

    // ===== and back the same way =====
    drive(&robot, "back", BACK_US, LIFT_UP_US, WIDTH_OUT_US)?;
    brake(&robot, LIFT_UP_US, WIDTH_OUT_US)?;

    // ===== hand it back =====
    // A four-channel frame, so the brake is released too.
    println!(
        "\n-- hand it back: straight, stopped, brake released, forks to {found_lift_us} / \
         {found_width_us} us --"
    );
    move_mast(&robot, found_lift_us, found_width_us)?;
    let end = robot.states().kin.lin_pos;
    println!(
        "ended {:.2} m from where it started (open loop, so not quite 0). Scene-authored: \
         left as it was found, never deleted.",
        distance(start, end)
    );
    Ok(())
}

/// Attach to `sys_id`, and make sure it really is a forklift.
///
/// `connect()` takes the robot type on trust: nothing in a state message names
/// it, and a truck at this id would attach just as happily. `z/properties` does
/// name it -- the kind of every actuator entry -- and once this handle has read
/// it (ex44 prints the rest), `forklift_actuators()` refuses a robot whose
/// layout is not `[wheel, servo, linear, linear]` instead of decoding its
/// report into wrong numbers.
fn attach(sys_id: u32) -> Result<VirtualRobot, VrError> {
    let robot = VirtualRobot::connect(RobotType::Forklift, Some(sys_id))?;
    robot.properties()?;
    robot.forklift_actuators()?;
    Ok(robot)
}

/// Stream one mast frame -- straight, stopped, brake released -- until the
/// forks are where the band says those pulses put them.
///
/// The frame latches, so sending it once would do; sending it every iteration
/// costs nothing and heals a frame lost on the way.
fn move_mast(robot: &VirtualRobot, lift_us: f64, width_us: f64) -> Result<(), VrError> {
    let want_lift = lift_m(lift_us);
    let want_width = width_m(width_us);
    let t0 = robot.states().elapsed;
    for i in 0..MAST_SAMPLES {
        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us)?;
        robot.rate(HZ);
        let a = robot.forklift_actuators()?;
        let t = robot.states().elapsed - t0;
        let there = (a.lift_height - want_lift).abs() < TOLERANCE_M
            && (a.fork_width - want_width).abs() < TOLERANCE_M;
        if there {
            println!("   t={t:5.2}s  {}  -- there", report(&a));
            return Ok(());
        }
        if i % 10 == 0 {
            println!("   t={t:5.2}s  {}", report(&a));
        }
    }
    println!(
        "   not there after {:.0} s: is the band or a range not the shipped one (ex44)?",
        f64::from(MAST_SAMPLES) / HZ
    );
    Ok(())
}

/// Hold one throttle, straight ahead, with the forks where they are.
///
/// A frame carries all four channels, so the mast pulses ride along in every
/// drive frame: one that left them out would move the forks as well.
fn drive(
    robot: &VirtualRobot,
    label: &str,
    throttle_us: f64,
    lift_us: f64,
    width_us: f64,
) -> Result<(), VrError> {
    println!(
        "\n-- {label}: throttle {throttle_us} us for {:.1} s --",
        f64::from(DRIVE_SAMPLES) / HZ
    );
    for i in 0..DRIVE_SAMPLES {
        robot.set_forklift(NEUTRAL_US, throttle_us, lift_us, width_us)?;
        robot.rate(HZ);
        if i % 10 == 9 {
            println!("   {}", report(&robot.forklift_actuators()?));
        }
    }
    Ok(())
}

/// Centre the throttle and add the fifth channel at full until the front axle
/// stops. The brake stays latched until the next four-channel frame.
fn brake(robot: &VirtualRobot, lift_us: f64, width_us: f64) -> Result<(), VrError> {
    let from = robot.forklift_actuators()?.drive_speed;
    let t0 = robot.states().elapsed;
    for _ in 0..BRAKE_SAMPLES {
        robot.set_forklift_with_brake(NEUTRAL_US, NEUTRAL_US, lift_us, width_us, BRAKE_FULL_US)?;
        robot.rate(HZ);
        let a = robot.forklift_actuators()?;
        if a.drive_speed.abs() < STOPPED_RAD_S {
            println!(
                "   brake {BRAKE_FULL_US} us: drive {from:+.2} -> {:+.2} rad/s in {:.2} s",
                a.drive_speed,
                robot.states().elapsed - t0
            );
            return Ok(());
        }
    }
    println!(
        "   still rolling after {:.0} s of full brake",
        f64::from(BRAKE_SAMPLES) / HZ
    );
    Ok(())
}

/// Where a lift pulse puts the carriage: linear across the shipped band, no
/// deadband.
fn lift_m(us: f64) -> f64 {
    LIFT_MIN_M + (us - 1100.0) / 800.0 * (LIFT_MAX_M - LIFT_MIN_M)
}

/// Where a fork-width pulse puts the forks, centre to centre: the same rule.
fn width_m(us: f64) -> f64 {
    WIDTH_MIN_M + (us - 1100.0) / 800.0 * (WIDTH_MAX_M - WIDTH_MIN_M)
}

/// The decoded report, in the units a person reads.
fn report(a: &ForkliftActuators) -> String {
    format!(
        "lift {:+.3} m  width {:.3} m  drive {:+5.2} rad/s ({:+.2} m/s)  steering {:+5.1} deg",
        a.lift_height,
        a.fork_width,
        a.drive_speed,
        a.drive_speed * TYRE_RADIUS_M,
        a.steering_angle.to_degrees()
    )
}

/// Straight-line distance between two positions, in whatever frame they share.
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The forklift's sys_id: the one command-line argument, or [`SYS_ID`].
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex37_hello_forklift".to_string());
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
