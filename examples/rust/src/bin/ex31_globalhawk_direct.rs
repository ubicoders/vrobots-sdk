//! ex31 -- globalhawk_direct: take the surfaces off the autopilot.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex31_globalhawk_direct -- <sys_id>
//! ```
//!
//! The RQ-4B Global Hawk is the first robot in this book that flies **itself**:
//! six aero panels and one engine, with onboard rate PIDs and an airspeed hold
//! closing the loop against whatever `SET_ANGVEL` setpoint arrives. Left alone it
//! cruises at 72.8 m/s and needs nothing from you.
//!
//! `set_fw_ctrl_mode(FW_DIRECT_SURFACE)` switches that off. The rate loop is
//! bypassed entirely and the six panels take your radians verbatim: **you are the
//! autopilot**, mixing included. This example proves that the per-panel path is
//! real, then shows the three ways it will surprise you.
//!
//! Scene-authored, like ex29 and ex30 -- `globalhawk` is not in any spawn
//! catalog, so attach by the `sys_id` ex11 lists. It lives in the IMU scene, not
//! the sandbox.
//!
//! # There is no mixer
//!
//! Each entry drives its own panel, in this order, in **radians**, clamped to the
//! airframe's 20-degree limit:
//!
//! | index | panel | what the onboard mixer does with it |
//! |---|---|---|
//! | 0 | left outboard flap | aileron, gain +1 |
//! | 1 | right outboard flap | aileron, gain -1 |
//! | 2 | **left inner flap** | **nothing -- gain 0 on all three channels** |
//! | 3 | **right inner flap** | **nothing** |
//! | 4 | rear left ruddervator | elevator -1, rudder +1 |
//! | 5 | rear right ruddervator | elevator -1, rudder -1 |
//!
//! Indices 2 and 3 are the proof. The simulator's own mixer has **zero gain**
//! there, so it can never move them: an inner flap that follows your command is a
//! deflection that could only have come through the per-panel path. The second
//! pose below deflects nothing else, on purpose.
//!
//! The length must equal the panel count exactly -- a wrong-length array makes
//! the simulator drop the **whole** command, never apply it partially.
//!
//! # The echo is the only receipt, and index 6 is not a pulse width
//!
//! `actuator.measured` has **panels + 1** entries:
//!
//! ```text
//! measured[0..=5]  per-panel deflection, RADIANS
//! measured[6]      the engine, NEWTONS -- not normalised, not a pulse width
//! ```
//!
//! `set_fw_thrust` takes newtons too (clamped to 20 kN), and `measured[6]` is the
//! only thing that ever confirms it. A simulator too old for this work publishes
//! **six** entries rather than seven -- that count is the version check.
//!
//! # Three things that will catch you out
//!
//! **Latching, with no watchdog.** Stop sending and the aircraft keeps flying
//! your last command forever, exactly like a dead PWM client on a multirotor.
//! The run below stops for two seconds to show the deflections not moving.
//!
//! **`reset()` reverts the mode.** Deliberately: keeping direct control with the
//! surface latches zeroed would relaunch the aircraft unflyable. So a
//! direct-surface client must re-assert `set_fw_ctrl_mode` after **every** reset,
//! and the aircraft you thought you were flying has quietly gone back to its
//! autopilot. Demonstrated, then undone.
//!
//! **Bumpless is not zeroed.** Entering direct mode seeds the latches from what
//! the plant is doing *now* -- including the thrust the airspeed hold happened to
//! be holding -- so nothing jolts. Which means a client that wants a particular
//! thrust must send `set_fw_thrust` **after every mode entry**, and after every
//! reset. Skip it and the engine keeps whatever the autopilot left, which is
//! usually about 3.8 kN at trim.
//!
//! `set_fw_thrust_bias` is not used here at all: it is an onboard-loop concept
//! and is ignored in direct mode.
//!
//! Scene-authored, so nothing is deleted. The aircraft is handed back to its
//! autopilot on the way out -- a Ctrl-C is not, and leaves it flying the last
//! deflections.

use vrobots_sdk::{RobotType, VirtualRobot, VrError, cmd};

const USAGE: &str = "\
The Global Hawk is scene-authored (IMU scene) and sys ids are handed out at
scene load and keep incrementing, so there is no id this file could hard-code.
Pass the live one:

    cargo run -p vrobots-examples --bin ex31_globalhawk_direct -- 15

List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const PANELS: usize = 6;
const DEFLECT_RAD: f64 = 0.15; // ~8.6 deg, inside the airframe's 20 deg limit
const CRUISE_N: f64 = 3800.0; // about what the airspeed hold carries at trim
const CLIMB_N: f64 = 8000.0; // a step big enough to be unmistakable
const HZ: f64 = 25.0;
const HOLD_SAMPLES: u32 = 60; // ~2.4 s per pose

/// Named poses, in panel order `[LF, RF, LIF, RIF, RLF, RRF]`.
const SWEEP: [(&str, [f64; PANELS]); 6] = [
    ("neutral", [0.0; PANELS]),
    // Only the inner flaps. The onboard mixer CANNOT produce this pose.
    (
        "inner flaps only",
        [0.0, 0.0, DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0],
    ),
    (
        "roll right",
        [DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0, 0.0, 0.0],
    ),
    ("nose up", [0.0, 0.0, 0.0, 0.0, -DEFLECT_RAD, -DEFLECT_RAD]),
    ("yaw right", [0.0, 0.0, 0.0, 0.0, DEFLECT_RAD, -DEFLECT_RAD]),
    ("neutral", [0.0; PANELS]),
];

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();
    let robot = VirtualRobot::connect(RobotType::GlobalHawk, Some(sys_id))?;

    let channels = robot.states().actuator.measured.len();
    println!(
        "attached to sys_id={} ({:?}), frame={:?}, {channels} actuator channels",
        robot.sys_id(),
        robot.robot_type(),
        robot.states().coord_frame_id
    );
    if channels != PANELS + 1 {
        println!(
            "WARNING: expected {} channels ({PANELS} panels + the engine). {channels} means a \
             different airframe -- or a simulator too old for the per-panel path, which \
             publishes {PANELS} and silently ignores SET_FW_SURFACES.",
            PANELS + 1
        );
    }

    // ===== hand the panels over =====
    // Order matters: mode first, then thrust. Entering direct mode inherits
    // whatever thrust the airspeed hold was carrying -- bumpless, not zeroed --
    // so the thrust command has to come AFTER the mode, every time.
    robot.set_fw_ctrl_mode(cmd::FW_DIRECT_SURFACE)?;
    robot.set_fw_thrust(CRUISE_N)?;
    println!("\nmode -> DIRECT_SURFACE, thrust -> {CRUISE_N} N\n");

    // ===== the sweep =====
    for (label, surfaces) in SWEEP {
        hold(&robot, label, &surfaces, Some(CRUISE_N))?;
    }

    // ===== the engine is in newtons =====
    hold(&robot, "thrust step", &[0.0; PANELS], Some(CLIMB_N))?;
    println!(
        "  measured[{PANELS}] = {:.0} N for a commanded {CLIMB_N} N -- newtons in, \
         newtons back, no pulse width anywhere.",
        thrust_of(&robot)
    );

    // ===== latching, and no watchdog =====
    println!("\n-- nothing sent for ~2 s --");
    robot.set_fw_surfaces(&[
        DEFLECT_RAD,
        -DEFLECT_RAD,
        DEFLECT_RAD,
        -DEFLECT_RAD,
        0.0,
        0.0,
    ])?;
    for i in 0..HOLD_SAMPLES {
        if i % 25 == 0 {
            print_echo(&robot, "  latched");
        }
        robot.rate(HZ);
    }
    println!("  unchanged. A command is a setpoint; there is no failsafe behind it.");

    // ===== reset takes the aircraft back =====
    println!("\n-- reset() --");
    robot.reset()?;
    for _ in 0..HOLD_SAMPLES {
        // Same command as before the reset, still being sent, and now ignored:
        // the mode went back to onboard, so the mixer is flying the panels.
        robot.set_fw_surfaces(&[
            DEFLECT_RAD,
            -DEFLECT_RAD,
            DEFLECT_RAD,
            -DEFLECT_RAD,
            0.0,
            0.0,
        ])?;
        robot.rate(HZ);
    }
    print_echo(&robot, "  after reset");
    println!(
        "  The inner flaps (2, 3) are back at 0 with the same command still \
         streaming: that is the mode reverting, not a dropped packet."
    );

    // ===== re-assert, which is all it takes =====
    println!("\n-- set_fw_ctrl_mode(DIRECT_SURFACE) again, then thrust again --");
    robot.set_fw_ctrl_mode(cmd::FW_DIRECT_SURFACE)?;
    robot.set_fw_thrust(CRUISE_N)?;
    hold(
        &robot,
        "recovered",
        &[0.0, 0.0, DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0],
        Some(CRUISE_N),
    )?;

    // ===== hand it back =====
    // Bumpless in this direction too: the rate PIDs reset and the airspeed hold
    // is seeded from the current thrust, so the aircraft does not sag.
    robot.set_fw_ctrl_mode(cmd::FW_ONBOARD_RATE)?;
    println!("\nmode -> ONBOARD_RATE. Scene-authored robot: left flying, never deleted.");
    Ok(())
}

/// Stream one pose for a while and print commanded against measured.
fn hold(
    robot: &VirtualRobot,
    label: &str,
    surfaces: &[f64],
    thrust_n: Option<f64>,
) -> Result<(), VrError> {
    println!("-- {label}: {} --", fmt(surfaces));
    for i in 0..HOLD_SAMPLES {
        // Latching means this does not have to be re-sent -- but a real
        // controller streams, so this one does too.
        robot.set_fw_surfaces(surfaces)?;
        if let Some(n) = thrust_n {
            robot.set_fw_thrust(n)?;
        }
        if i % 30 == 0 {
            print_echo(robot, "  ");
        }
        robot.rate(HZ);
    }
    Ok(())
}

/// The actuator echo: panels in radians, then the engine in newtons.
fn print_echo(robot: &VirtualRobot, prefix: &str) {
    let s = robot.states();
    let m = &s.actuator.measured;
    let panels = m.len().saturating_sub(1);
    println!(
        "{prefix} t={:7.2}s panels={} engine={:8.0} N  rates=({:+6.2},{:+6.2},{:+6.2}) deg/s",
        s.elapsed,
        fmt(&m[..panels]),
        m.last().copied().unwrap_or(0.0),
        s.kin.ang_vel[0].to_degrees(),
        s.kin.ang_vel[1].to_degrees(),
        s.kin.ang_vel[2].to_degrees()
    );
}

/// The engine channel, newtons.
fn thrust_of(robot: &VirtualRobot) -> f64 {
    robot
        .states()
        .actuator
        .measured
        .last()
        .copied()
        .unwrap_or(0.0)
}

/// Six deflections, aligned so two rows can be compared by eye.
fn fmt(v: &[f64]) -> String {
    let body = v
        .iter()
        .map(|x| format!("{x:+6.3}"))
        .collect::<Vec<_>>()
        .join(",");
    format!("[{body}]")
}

/// The scene-authored aircraft's sys_id, from the one command-line argument.
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex31_globalhawk_direct".to_string());
    match args.next().map(|a| a.parse::<u32>()) {
        Some(Ok(id)) => id,
        _ => {
            eprintln!("usage: {program} <sys_id>\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
