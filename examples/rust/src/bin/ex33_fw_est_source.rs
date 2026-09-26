//! ex33 -- fw_est_source: let the autopilot believe your estimator instead of the truth.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex33_fw_est_source -- <sys_id>
//! ```
//!
//! Every control loop in the simulator is fed the **exact** attitude, because it
//! is a simulator and it can be. `SET_FW_EST_SOURCE` (command id 311) is the one
//! switch that takes that away:
//!
//! | value | the onboard loop is fed |
//! |---|---|
//! | `cmd::FW_EST_TRUTH` (0, the default) | the simulator's true attitude |
//! | `cmd::FW_EST_OBSERVER` (1) | whatever is published on the robot's `z/estimate` topic |
//!
//! The controller itself never knows which. The swap happens upstream of it, in
//! the robot -- which is exactly how a real flight computer gets fooled, and the
//! whole point of the experiment: **a fooled autopilot is just an autopilot with
//! a lying sensor.** Write an attitude estimator, publish it, and the aircraft
//! flies on your errors.
//!
//! Two properties make it safe to try:
//!
//! **It only affects the onboard loop.** `FW_DIRECT_SURFACE` (ex31, ex32) never
//! consults an attitude at all, so the switch is inert there. This example
//! therefore stays in `FW_ONBOARD_RATE` throughout.
//!
//! **A stale estimate falls back to truth.** An estimate older than **0.5 s** is
//! not trusted, and the loop silently reverts to truth until a fresh one arrives
//! (the simulator logs a warning saying so). And like the control mode,
//! `reset()` puts the source back to truth.
//!
//! # What this run can and cannot show you
//!
//! Nothing publishes `z/estimate` here, so selecting the observer is a
//! **demonstration of the fallback**, not of the swap: the loop asks for an
//! estimate, finds none within the staleness window, and keeps flying on truth.
//! The evidence is that the tracking error does not change -- printed below,
//! across all three phases.
//!
//! The swap itself is ex35, which selects the observer *and* publishes on:
//!
//! ```text
//! vrobots/{sys_id}/z/estimate      swarmbotix.states.EstimateState
//! ```
//!
//! `publish_estimate` and `publish_estimate_euler` build that message for you --
//! `estimate.valid`, `estimate.kinematics.pose.orientation`, frame-tagged from
//! the connect header -- and one of ex35's phases publishes a pitch 5 degrees
//! above the true one, so the aircraft visibly flies the wrong attitude. An
//! estimate does not latch the way a command does: the simulator ages it from
//! arrival and stops trusting it after 0.5 s, so a publisher has to repeat
//! itself at 20 Hz or better.
//!
//! # Why yaw
//!
//! The onboard loop tracks `SET_ANGVEL`, so this example publishes one and
//! measures how well it is followed. It commands **yaw** rate specifically:
//! roll and pitch demands are summed with a wings-level assist and an
//! altitude-hold trim, so their steady tracking error is not zero and would make
//! a poor yardstick. Yaw has no assist -- the rate loop's integrator drives its
//! error to zero -- so it measures the loop and nothing else.
//!
//! `SET_ANGVEL` has a typed wrapper, `set_angvel` -- ex35 uses it -- but this
//! example keeps the generic `send_cmd` path on purpose: it is ex08's escape
//! hatch, used in anger, and the two spell the same bytes. It carries a
//! **vec3**, so unlike the surface array it *is* re-expressed from your header
//! frame into the robot's, as an axial vector: the connect below stamps `"frd"`
//! so that `[0, 0, r]` means what it looks like.
//!
//! And like everything else on this aircraft the setpoint **latches** -- except
//! across a `reset()`, which clears it. Phase 3 re-sends it for that reason.
//!
//! Scene-authored (IMU scene): attach by `sys_id`, never delete.

use vrobots_sdk::{Axes, CmdArgs, ConnectOptions, RobotType, VirtualRobot, VrError, cmd};

const USAGE: &str = "\
The Global Hawk is scene-authored (IMU scene) and sys ids are handed out at
scene load and keep incrementing, so there is no id this file could hard-code.
Pass the live one:

    cargo run -p vrobots-examples --bin ex33_fw_est_source -- 15

List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const YAW_RATE: f64 = 0.05; // rad/s, nose right -- gentle, and well inside authority
const HZ: f64 = 25.0;
const SETTLE_SAMPLES: u32 = 100; // ~4 s for the integrator to null the error
const MEASURE_SAMPLES: u32 = 150; // ~6 s averaged

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();

    // The rate setpoint is a vec3 and IS converted from this frame into the
    // robot's. Stamping the aircraft's own frame makes [0, 0, r] mean r.
    let robot = VirtualRobot::connect_with(
        RobotType::GlobalHawk,
        Some(sys_id),
        ConnectOptions::default().with_frame("frd", Axes::FRD),
    )?;
    println!(
        "attached to sys_id={} ({:?}), frame={:?}",
        robot.sys_id(),
        robot.robot_type(),
        robot.states().coord_frame_id
    );

    // This switch is an onboard-loop concept, so make sure the onboard loop is
    // the one flying. Bumpless, and it does not teleport the aircraft.
    robot.set_fw_ctrl_mode(cmd::FW_ONBOARD_RATE)?;
    println!("mode -> ONBOARD_RATE (the only mode where an attitude is consulted at all)\n");

    // ===== phase 1: truth, the default =====
    robot.set_fw_est_source(cmd::FW_EST_TRUTH)?;
    let truth = track(&robot, "truth (source 0)")?;

    // ===== phase 2: observer, with nobody publishing an estimate =====
    robot.set_fw_est_source(cmd::FW_EST_OBSERVER)?;
    println!(
        "\nsource -> OBSERVER. Nothing publishes z/estimate here, so within 0.5 s \
         the estimate is stale and the loop is fed truth again -- the simulator \
         logs a warning saying exactly that."
    );
    let observer = track(&robot, "observer (source 1), no publisher")?;

    // ===== phase 3: reset puts it back =====
    println!("\n-- reset() --");
    robot.reset()?;
    println!(
        "  the source is back to truth and the mode back to onboard, and the \
         SET_ANGVEL latch was cleared -- so it has to be re-sent."
    );
    let after_reset = track(&robot, "after reset")?;

    println!("\nsteady yaw-rate tracking, commanded {YAW_RATE} rad/s:");
    for run in [&truth, &observer, &after_reset] {
        println!(
            "  {:<34} mean r={:+7.4} rad/s  mean error={:+7.4}  |error|={:6.4}",
            run.label, run.mean_rate, run.mean_error, run.mean_abs_error
        );
    }
    println!(
        "Phase 2 matching phase 1 IS the fallback: the loop asked for an estimate, \
         found none fresh, and kept flying on truth."
    );

    // ===== the value the SDK will not send =====
    match robot.set_fw_est_source(2) {
        Ok(()) => println!("\nUNEXPECTED: source 2 was accepted"),
        Err(e) => println!("\nset_fw_est_source(2) -> [{}] {}", e.code(), e.detail()),
    }

    // ===== hand it back =====
    robot.send_cmd(cmd::SET_ANGVEL, &CmdArgs::vector([0.0; 3]))?;
    println!("rate setpoint zeroed. Scene-authored robot: left flying, never deleted.");
    Ok(())
}

/// One tracking window.
struct Tracking {
    label: String,
    mean_rate: f64,
    mean_error: f64,
    mean_abs_error: f64,
}

/// Stream a steady yaw-rate setpoint, let the loop settle, then average the
/// error over a window.
fn track(robot: &VirtualRobot, label: &str) -> Result<Tracking, VrError> {
    println!("-- {label} --");
    let demand = [0.0, 0.0, YAW_RATE]; // FRD [p, q, r]

    for _ in 0..SETTLE_SAMPLES {
        // Deliberately the generic path (ex08), not set_angvel: same bytes,
        // demonstrated once here. The vec3 field is the one this id reads;
        // everything else stays off the wire.
        robot.send_cmd(cmd::SET_ANGVEL, &CmdArgs::vector(demand))?;
        robot.rate(HZ);
    }

    let (mut rate_sum, mut error_sum, mut abs_sum) = (0.0, 0.0, 0.0);
    for i in 0..MEASURE_SAMPLES {
        robot.send_cmd(cmd::SET_ANGVEL, &CmdArgs::vector(demand))?;
        let s = robot.states();
        let r = s.kin.ang_vel[2]; // FRD: the third body rate is yaw
        rate_sum += r;
        error_sum += YAW_RATE - r;
        abs_sum += (YAW_RATE - r).abs();
        if i % 50 == 0 {
            println!(
                "   t={:7.2}s r={:+7.4} rad/s  err={:+7.4}  bank rate p={:+7.4}  \
                 engine={:8.0} N",
                s.elapsed,
                r,
                YAW_RATE - r,
                s.kin.ang_vel[0],
                s.actuator.measured.last().copied().unwrap_or(0.0)
            );
        }
        robot.rate(HZ);
    }

    let n = f64::from(MEASURE_SAMPLES);
    Ok(Tracking {
        label: label.to_string(),
        mean_rate: rate_sum / n,
        mean_error: error_sum / n,
        mean_abs_error: abs_sum / n,
    })
}

/// The scene-authored aircraft's sys_id, from the one command-line argument.
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex33_fw_est_source".to_string());
    match args.next().map(|a| a.parse::<u32>()) {
        Some(Ok(id)) => id,
        _ => {
            eprintln!("usage: {program} <sys_id>\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
