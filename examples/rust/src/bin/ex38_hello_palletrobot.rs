//! ex38 -- hello_palletrobot: four wheel channels, no twist command, and the mix is yours.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex38_hello_palletrobot          # the warehouse pallet robot, sys_id 1
//! cargo run -p vrobots-examples --bin ex38_hello_palletrobot -- <id>  # one at another id
//! ```
//!
//! The pallet robot is the low mecanum robot that drives under a pedestal and
//! lifts it on a turning pad. The warehouse scene pins it at `sys_id` 1.
//! `SET_PALLETROBOT` is six pulse widths, in one order for the command and the
//! actuator report alike (`palletrobot_channel::*`):
//!
//! | index | channel | 1100 us | 1500 us | 1900 us |
//! |---|---|---|---|---|
//! | 0-3 | wheel ESC FL, FR, RL, RR | -28 rad/s | stop | +28 rad/s, rolling forward |
//! | 4 | pad lift, a position servo | 0 m | 0.03 m | 0.06 m |
//! | 5 | pad turn, a rate | -90 deg/s | hold | +90 deg/s, clockwise from above |
//!
//! # The robot takes no twist
//!
//! There is no "go right at 0.3 m/s" command. Each wheel has its own ESC, and
//! turning a body twist into four wheel speeds -- the mecanum mix -- is the
//! client's job. It is a few lines of arithmetic, in `mix()` below, for rollers
//! in the standard X layout, on the robot's virtual geometry R = 0.054 m,
//! lx = 0.35 m and ly = 0.30 m:
//!
//! ```text
//! FL = (vx + vy + (lx+ly) wz) / R        FR = (vx - vy - (lx+ly) wz) / R
//! RL = (vx - vy + (lx+ly) wz) / R        RR = (vx + vy - (lx+ly) wz) / R
//! us = round(1500 + 400 * w / 28)        for each wheel speed w in rad/s
//! ```
//!
//! `vx` is forward and `vy` right in m/s, `wz` clockwise seen from above in
//! rad/s: FRU, the frame this robot reports in unless `srv/frames` changed it,
//! so the state's `kin.lin_vel[0]`, `kin.lin_vel[1]` and `kin.ang_vel[2]` are the
//! same three numbers coming back. The wheel channels themselves are physical
//! (+ rolls forward) and are never frame-converted. If any wheel would pass 28 rad/s, all four scale down by the
//! same factor: clipping one alone changes the direction of travel.
//!
//! The geometry is virtual -- the model has no rollers -- and nothing on the
//! wire reads it back, so a client keeps its own copy. If `srv/palletrobot` has
//! changed it, the constants below must follow.
//!
//! # Deadband, ramps, and what confirms a move
//!
//! The wheels and the pad turn have a +/-20 us deadband, which makes about
//! 0.08 m/s the slowest controllable speed; the pad lift has none. The wheels
//! ramp at 60 rad/s^2 and the pad climbs at 0.03 m/s, so the actuator report,
//! `palletrobot_actuators()`, is where a move is confirmed. Its wheel speeds are
//! the motors' own: a blocked robot still reports them, and `kin` is what the
//! body actually did.
//!
//! # Leaving it as found
//!
//! Every move is undone by its mirror image -- strafe right then left, turn one
//! way then back -- so the robot ends where it started, to within the open-loop
//! error. The pad returns to the pulse it was found at, and the last frame
//! stops the wheels; with all four stopped the robot holds where it parked. Let
//! it run to the end: a Ctrl-C leaves the last frame latched. Scene-authored:
//! attached by `sys_id`, never deleted.

use vrobots_sdk::{PalletRobotActuators, RobotType, VirtualRobot, VrError, palletrobot_channel};

const USAGE: &str = "\
The warehouse scene pins its pallet robot at sys_id 1, which is the default. For
one at another id, pass it:

    cargo run -p vrobots-examples --bin ex38_hello_palletrobot -- 1

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const SYS_ID: u32 = 1; // the warehouse scene's pallet robot

// The virtual mecanum geometry: the shipped srv/palletrobot values.
const WHEEL_RADIUS_M: f64 = 0.054; // R
const HALF_WHEELBASE_M: f64 = 0.35; // lx, centre to the front and rear axles
const HALF_TRACK_M: f64 = 0.30; // ly, centre to the left and right wheels
const MAX_WHEEL_RAD_S: f64 = 28.0; // a wheel at 400 us off neutral
const PAD_MAX_M: f64 = 0.06; // the pad at the top of the band

const NEUTRAL_US: f64 = 1500.0; // a wheel stopped, the pad turn holding
const PAD_TOP_US: f64 = 1900.0;
const STRAFE_M_S: f64 = 0.3;
const TURN_RAD_S: f64 = 0.5; // ~29 deg/s
const HZ: f64 = 25.0;
const MOVE_SAMPLES: u32 = 50; // 2 s per move
const PAD_SAMPLES: u32 = 100; // 4 s; the whole 0.06 m takes 2
const PAD_TOLERANCE_M: f64 = 0.001;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();
    let robot = match attach(sys_id) {
        Ok(robot) => robot,
        Err(e) => {
            eprintln!(
                "no pallet robot at sys_id {sys_id}: [{}] {e}\n\n{USAGE}",
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
    // The echo is the latch, in command order, so the pad's pulse is read off
    // it here and sent back at the end.
    let found_pad_us = f64::from(
        s.actuator
            .pwm
            .get(palletrobot_channel::PAD_LIFT)
            .copied()
            .unwrap_or(1100),
    );
    println!(
        "found: echo {:?} = [FL, FR, RL, RR, pad lift, pad turn] us",
        s.actuator.pwm
    );
    println!("       {}", report(&robot.palletrobot_actuators()?));

    // ===== strafe right, then back =====
    drive(&robot, "strafe right", 0.0, STRAFE_M_S, 0.0, found_pad_us)?;
    drive(&robot, "strafe back", 0.0, -STRAFE_M_S, 0.0, found_pad_us)?;

    // ===== turn on the spot, clockwise, then back =====
    drive(&robot, "turn clockwise", 0.0, 0.0, TURN_RAD_S, found_pad_us)?;
    drive(&robot, "turn back", 0.0, 0.0, -TURN_RAD_S, found_pad_us)?;

    // ===== stop =====
    // A zero twist mixes to 1500 on every wheel: a stop is just another frame.
    drive(&robot, "stop", 0.0, 0.0, 0.0, found_pad_us)?;

    // ===== the pad: up to the top, then back =====
    pad(&robot, PAD_TOP_US)?;
    pad(&robot, found_pad_us)?;

    // ===== hand it back =====
    // pad() ends on a frame with the wheels stopped and the pad turn holding,
    // and that frame stays latched.
    println!(
        "\nwheels stopped, pad back at {found_pad_us} us. Scene-authored: left as it was \
         found, never deleted."
    );
    Ok(())
}

/// Attach to `sys_id`, and make sure it really is a pallet robot.
///
/// `connect()` takes the robot type on trust; `z/properties` names it. Once
/// this handle has read it (ex44), `palletrobot_actuators()` refuses a robot
/// whose layout is not four wheels, a linear pad lift and a servo pad turn.
fn attach(sys_id: u32) -> Result<VirtualRobot, VrError> {
    let robot = VirtualRobot::connect(RobotType::PalletRobot, Some(sys_id))?;
    robot.properties()?;
    robot.palletrobot_actuators()?;
    Ok(robot)
}

/// Mix one body twist into wheel pulses and hold it for [`MOVE_SAMPLES`].
///
/// The pad pulse rides along in every frame: a frame is all six channels, and
/// one that left the pad out would move it.
fn drive(
    robot: &VirtualRobot,
    label: &str,
    vx: f64,
    vy: f64,
    wz: f64,
    pad_us: f64,
) -> Result<(), VrError> {
    let w = mix(vx, vy, wz);
    let us = [to_us(w[0]), to_us(w[1]), to_us(w[2]), to_us(w[3])];
    println!("\n-- {label}: vx {vx:+.2} m/s, vy {vy:+.2} m/s, wz {wz:+.2} rad/s --");
    println!(
        "   mix: [FL, FR, RL, RR] = [{:+.2}, {:+.2}, {:+.2}, {:+.2}] rad/s \
         -> [{:.0}, {:.0}, {:.0}, {:.0}] us",
        w[0], w[1], w[2], w[3], us[0], us[1], us[2], us[3]
    );
    for i in 0..MOVE_SAMPLES {
        robot.set_palletrobot(us[0], us[1], us[2], us[3], pad_us, NEUTRAL_US)?;
        robot.rate(HZ);
        if i % 25 == 24 {
            // FRU: lin_vel is [forward, right, up], ang_vel[2] clockwise from above.
            let s = robot.states();
            println!(
                "   {}  body vx {:+.2} vy {:+.2} m/s, wz {:+.2} rad/s",
                report(&robot.palletrobot_actuators()?),
                s.kin.lin_vel[0],
                s.kin.lin_vel[1],
                s.kin.ang_vel[2]
            );
        }
    }
    Ok(())
}

/// Move the pad to `pad_us`, wheels stopped, until the report says it is there.
fn pad(robot: &VirtualRobot, pad_us: f64) -> Result<(), VrError> {
    // A position servo with no deadband: linear from 0 m at 1100 to the top at 1900.
    let want = PAD_MAX_M * (pad_us - 1100.0) / 800.0;
    println!("\n-- pad lift {pad_us} us: {want:.3} m by the band --");
    let t0 = robot.states().elapsed;
    for i in 0..PAD_SAMPLES {
        robot.set_palletrobot(
            NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, pad_us, NEUTRAL_US,
        )?;
        robot.rate(HZ);
        let a = robot.palletrobot_actuators()?;
        let t = robot.states().elapsed - t0;
        if (a.pad_height - want).abs() < PAD_TOLERANCE_M {
            println!("   t={t:5.2}s  {}  -- there", report(&a));
            return Ok(());
        }
        if i % 10 == 0 {
            println!("   t={t:5.2}s  {}", report(&a));
        }
    }
    println!(
        "   not there after {:.0} s: has srv/palletrobot changed the pad?",
        f64::from(PAD_SAMPLES) / HZ
    );
    Ok(())
}

/// The mecanum mix: a body twist into four wheel speeds, rad/s, `[FL, FR, RL, RR]`.
///
/// `vx` forward and `vy` right in m/s, `wz` clockwise from above in rad/s.
fn mix(vx: f64, vy: f64, wz: f64) -> [f64; 4] {
    let k = HALF_WHEELBASE_M + HALF_TRACK_M; // lx + ly
    let mut wheels = [
        (vx + vy + k * wz) / WHEEL_RADIUS_M,
        (vx - vy - k * wz) / WHEEL_RADIUS_M,
        (vx - vy + k * wz) / WHEEL_RADIUS_M,
        (vx + vy - k * wz) / WHEEL_RADIUS_M,
    ];

    // Too fast for one wheel: slow all four by the same factor, which keeps the
    // direction of travel. Clipping only the fast one would bend it.
    let mut fastest = 0.0_f64;
    for w in wheels {
        fastest = fastest.max(w.abs());
    }
    if fastest > MAX_WHEEL_RAD_S {
        for w in &mut wheels {
            *w *= MAX_WHEEL_RAD_S / fastest;
        }
    }
    wheels
}

/// One wheel speed as the pulse that asks for it: 400 us either side of neutral
/// is [`MAX_WHEEL_RAD_S`].
///
/// Rounded to a whole microsecond, which is all the wire carries. The SDK rounds
/// to the nearest one as well, so this only makes the printed pulse the one that
/// goes out.
fn to_us(rad_per_s: f64) -> f64 {
    (NEUTRAL_US + 400.0 * rad_per_s / MAX_WHEEL_RAD_S).round()
}

/// The decoded report, in the units a person reads.
fn report(a: &PalletRobotActuators) -> String {
    let [fl, fr, rl, rr] = a.wheel_speeds;
    format!(
        "wheels [{fl:+5.1}, {fr:+5.1}, {rl:+5.1}, {rr:+5.1}] rad/s  pad {:.3} m  pad angle {:+.1} deg",
        a.pad_height,
        a.pad_angle.to_degrees()
    )
}

/// The pallet robot's sys_id: the one command-line argument, or [`SYS_ID`].
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex38_hello_palletrobot".to_string());
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
