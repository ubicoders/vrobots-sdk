//! ex35 -- publish_estimate: hand the autopilot an attitude you made up.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex35_publish_estimate -- <sys_id>
//! ```
//!
//! ex33 selected the observer with nothing on `z/estimate`, and got the fallback:
//! the loop asked for an estimate, found none fresh, and kept flying on truth.
//! This is the other half. [`publish_estimate`] puts a
//! `swarmbotix.states.EstimateState` on that topic, so
//! `set_fw_est_source(FW_EST_OBSERVER)` finally has something to select -- and
//! the aircraft flies on **your** attitude, errors and all.
//!
//! Four phases, each measured the same way so the numbers can be compared:
//!
//! | phase | source | published on `z/estimate` | what to expect |
//! |---|---|---|---|
//! | 1 | truth | the robot's own quaternion | nothing changes; the `_Est` gauges move |
//! | 2 | observer | the robot's own quaternion | still nothing: your estimate is right |
//! | 3 | observer | the same, pitched up 5 deg | the nose trims down by about 5 deg |
//! | 4 | observer | nothing at all | 0.5 s later the loop is back on truth |
//!
//! Phase 1 is what makes phase 3 mean anything. A truth-copy estimator is the one
//! estimator whose error is exactly zero, so a difference between phases 1 and 2
//! would be the plumbing rather than the estimate, and there is nowhere left for
//! phase 3's shift to have come from.
//!
//! # The setpoint latches; the estimate does not
//!
//! [`set_angvel`] is sent **once**, at the top, and stands for the whole run --
//! that is what a command is, and ex33's `send_cmd(SET_ANGVEL, ..)` escape hatch
//! is retired now that there is a typed wrapper for it. The estimate is the
//! opposite: the simulator ages it **from arrival**, in sim time, and stops
//! trusting it after 0.5 s, so it has to be republished every iteration at 20 Hz
//! or better. Phase 4 is that difference made visible -- the loop stops
//! publishing and sends nothing else, and the aircraft keeps the rate setpoint
//! while losing the attitude.
//!
//! # What the lie actually does
//!
//! The onboard loop adds two attitude assists on top of the rate setpoint, and
//! both read roll and pitch out of the **believed** attitude: a wings-leveller,
//! and an altitude hold that biases the pitch demand. Tell it the nose is 5
//! degrees higher than it is and the pitch assist trims 5 degrees of nose-down to
//! "correct" it -- at the airframe's 1.5 rad/s per rad that is an extra 7.5 deg/s
//! of nose-down demand while it settles, and the steady state is a true pitch
//! about 5 degrees below where phase 2 held it. The aircraft then sinks until the
//! altitude hold has bought those 5 degrees back, which takes tens of metres.
//!
//! **The rate loop is never wrong.** It tracks its demand as well in phase 3 as
//! in phase 1; it is being asked for the wrong thing. That is the whole lesson: a
//! fooled autopilot is just an autopilot with a lying sensor.
//!
//! Five degrees is deliberately small. The altitude assist is clamped at 10
//! degrees, so a lie inside the clamp settles the aircraft lower instead of
//! departing -- raise `PITCH_BIAS_DEG` past 10 and it cannot recover.
//!
//! # `valid`, and the two ways of saying nothing
//!
//! Everything here publishes `valid = true`. `false` is not a status flag: the
//! simulator drops the message before it reads the quaternion **and does not
//! reset the age counter**, so a stream of invalid estimates is
//! indistinguishable, sim-side, from phase 4's silence. If your filter has not
//! converged that is still the honest thing to send -- just do not expect to be
//! able to tell it apart from a dead publisher.
//!
//! # The rates ride along, and nothing reads them
//!
//! `Some(s.sensors.gyroscope.angular_velocity)` fills the estimate's `twist`,
//! because a filter that has rates should say so and the field is there. **No
//! consumer reads it today** -- the fixed wing takes the quaternion and nothing
//! else. `None` would leave `twist` off the wire entirely, which is a different
//! statement from sending zeros.
//!
//! # The frame is the header's
//!
//! The SDK leaves the estimate's own frame pair unset, so it inherits the
//! header's -- the `"frd"` this connect stamps. That is why the truth copy can be
//! `s.kin.quat` straight off the state stream: this aircraft reports in `frd`
//! too, so both ends of the round trip name the same convention. The run checks
//! that rather than assuming it.
//!
//! Scene-authored (IMU scene): attach by `sys_id`, never delete.
//!
//! [`publish_estimate`]: vrobots_sdk::VirtualRobot::publish_estimate
//! [`set_angvel`]: vrobots_sdk::VirtualRobot::set_angvel

use vrobots_sdk::rotations::{self, EulerOrder};
use vrobots_sdk::{Axes, ConnectOptions, RobotType, State, VirtualRobot, VrError, cmd};

const USAGE: &str = "\
The Global Hawk is scene-authored (IMU scene) and sys ids are handed out at
scene load and keep incrementing, so there is no id this file could hard-code.
Pass the live one:

    cargo run -p vrobots-examples --bin ex35_publish_estimate -- 15

List what is publishing with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

/// The aircraft's frame, and therefore the order its angles decompose in.
const FRAME_ID: &str = "frd";
const EULER_ORDER: EulerOrder = EulerOrder::Zyx;

/// The rate setpoint, FRD `[p, q, r]` in rad/s. Sent once: it latches.
const RATE_SETPOINT: [f64; 3] = [0.0, 0.0, 0.0];

/// How much nose-up the phase-3 estimate invents, degrees. Under the onboard
/// loop's 10-degree altitude-assist clamp, so the aircraft sags rather than
/// departs.
const PITCH_BIAS_DEG: f64 = 5.0;

const HZ: f64 = 25.0; // the publish rate; the sim wants 20 Hz or better
const STALE_S: f64 = 0.5; // the sim's own staleness window, aged from arrival
const SETTLE_SAMPLES: u32 = 100; // ~4 s for the assists and the rate loop to settle
const MEASURE_SAMPLES: u32 = 150; // ~6 s averaged
const REPORT_EVERY: u32 = 50; // a progress line every ~2 s

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();

    // The estimate inherits this header frame, and the rate setpoint is
    // re-expressed out of it, so stamping the aircraft's own makes both mean
    // what they look like.
    let robot = VirtualRobot::connect_with(
        RobotType::GlobalHawk,
        Some(sys_id),
        ConnectOptions::default().with_frame(FRAME_ID, Axes::FRD),
    )?;
    println!(
        "attached to sys_id={} ({:?}), frame={:?}",
        robot.sys_id(),
        robot.robot_type(),
        robot.states().coord_frame_id
    );
    println!(
        "publishing estimates on {}",
        vrobots_sdk::topics::estimate(sys_id)
    );

    // The truth copy below is `s.kin.quat`, which is in the robot's reporting
    // frame, published under a header that says FRAME_ID. If those two ever
    // disagree the simulator re-bases a quaternion that was already correct.
    let reported = robot.states().coord_frame_id.clone();
    if reported != FRAME_ID {
        println!(
            "NOTE: this robot reports in {reported:?} and this run stamps {FRAME_ID:?}. \
             The truth copy would be re-based on arrival -- convert it first, or connect \
             with the robot's own frame."
        );
    }

    // The estimate source is an onboard-loop concept, so make sure the onboard
    // loop is the one flying. Bumpless, and it does not teleport the aircraft.
    robot.set_fw_ctrl_mode(cmd::FW_ONBOARD_RATE)?;

    // Sent ONCE. A command latches: this setpoint is still in force forty
    // seconds from now, including through phase 4's silence.
    robot.set_angvel(RATE_SETPOINT)?;
    println!(
        "mode -> ONBOARD_RATE, SET_ANGVEL -> [{:.1}, {:.1}, {:.1}] rad/s (sent once -- it latches)",
        RATE_SETPOINT[0], RATE_SETPOINT[1], RATE_SETPOINT[2]
    );

    // Both entry points build the same message. This is the Euler one, used
    // once: the aircraft's own attitude, decomposed in the frame's order and
    // rebuilt by the SDK on the way out.
    let s = robot.states();
    let euler = rotations::quat_to_euler(s.kin.quat, EULER_ORDER);
    robot.publish_estimate_euler(
        euler,
        EULER_ORDER,
        Some(s.sensors.gyroscope.angular_velocity),
        true,
    )?;
    println!(
        "publish_estimate_euler once: roll/pitch/yaw = ({:+.2},{:+.2},{:+.2}) deg in order {} \
         -- the same wire message publish_estimate builds, from angles instead of a quaternion\n",
        euler[0].to_degrees(),
        euler[1].to_degrees(),
        euler[2].to_degrees(),
        EULER_ORDER.name()
    );

    // ===== phase 1: the control -- a perfect estimator nobody is listening to =====
    robot.set_fw_est_source(cmd::FW_EST_TRUTH)?;
    println!("source -> TRUTH. Publishing anyway: the _Est gauges move, the flight does not.");
    let truth = track(
        &robot,
        "1 truth source, truth-copy estimate",
        Estimator::TruthCopy,
    )?;

    // ===== phase 2: the swap, with an estimate that happens to be right =====
    robot.set_fw_est_source(cmd::FW_EST_OBSERVER)?;
    println!(
        "\nsource -> OBSERVER. Same publisher, same quaternion -- but the loop is flying \
         YOUR attitude now, and there is no field anywhere that says so."
    );
    let observer = track(
        &robot,
        "2 observer source, truth-copy estimate",
        Estimator::TruthCopy,
    )?;

    // ===== phase 3: the same loop, flying a lie =====
    println!(
        "\nsame source, same publisher, +{PITCH_BIAS_DEG:.1} deg of pitch composed onto \
         every estimate. The aircraft is about to be told its nose is higher than it is."
    );
    let lie = track(
        &robot,
        &format!("3 observer source, +{PITCH_BIAS_DEG:.1} deg pitch lie"),
        Estimator::PitchBias,
    )?;
    println!(
        "  against phase 2: {:+.2} deg of true pitch, {:+.1} m of altitude. The lie was \
         +{PITCH_BIAS_DEG:.1} deg.",
        lie.mean_pitch_deg - observer.mean_pitch_deg,
        lie.altitude_m - observer.altitude_m
    );

    // ===== put the aircraft back where phase 1 found it =====
    // Phase 4 is only comparable from a level, on-altitude start, and the lie
    // spent tens of metres. Reset relaunches at trim -- and takes the estimate
    // source and the SET_ANGVEL latch with it, so both are re-sent.
    println!("\n-- reset() --");
    robot.reset()?;
    robot.set_fw_est_source(cmd::FW_EST_OBSERVER)?;
    robot.set_angvel(RATE_SETPOINT)?;
    println!(
        "  relaunched at trim, and the source and the rate setpoint re-sent: reset() clears \
         both."
    );

    // ===== phase 4: the fallback, from the publisher's side this time =====
    println!(
        "\nstill OBSERVER, and nothing published from here on. After {STALE_S} s of silence \
         the estimate is stale and the loop is fed truth again -- the simulator logs a \
         warning saying exactly that."
    );
    let stale = track(
        &robot,
        "4 observer source, nothing published",
        Estimator::Silent,
    )?;

    // ===== the four numbers =====
    let window_s = f64::from(MEASURE_SAMPLES) / HZ;
    println!("\nwhat each phase flew on, and what the airframe did about it:");
    for run in [&truth, &observer, &lie, &stale] {
        println!(
            "  {:<38} pitch={:+6.2} deg  roll={:+6.2} deg  r={:+7.4} rad/s  \
             alt={:9.1} m ({:+7.1} m over {window_s:.1} s)",
            run.label,
            run.mean_pitch_deg,
            run.mean_roll_deg,
            run.mean_yaw_rate,
            run.altitude_m,
            run.climb_m
        );
    }
    println!(
        "Phase 2 matching phase 1 is the swap working: the loop flew your estimate and your \
         estimate was right. Phase 3 is the same loop, the same gains and the same setpoint, \
         given one wrong number. Phase 4 is the staleness clock handing it back."
    );

    // ===== hand it back =====
    robot.set_fw_est_source(cmd::FW_EST_TRUTH)?;
    robot.set_angvel(RATE_SETPOINT)?;
    println!(
        "source -> TRUTH, rate setpoint zeroed. Scene-authored robot: left flying, never deleted."
    );
    Ok(())
}

/// What the publisher does during one tracking window.
#[derive(Clone, Copy)]
enum Estimator {
    /// The robot's own attitude, published straight back at it. Zero error by
    /// construction, which is what makes it the control.
    TruthCopy,
    /// The same attitude with [`PITCH_BIAS_DEG`] of nose-up composed on.
    PitchBias,
    /// Nothing goes on the wire.
    Silent,
}

/// One measurement window.
struct Tracking {
    label: String,
    mean_pitch_deg: f64,
    mean_roll_deg: f64,
    mean_yaw_rate: f64,
    /// Altitude at the end of the window, metres.
    altitude_m: f64,
    /// How much of that was gained or lost across the window, metres.
    climb_m: f64,
}

/// Publish for a while, let the assists settle, then average over a window.
///
/// Identical in every phase so the four rows can be read side by side: only
/// `estimator` changes, and in phase 4 not even that -- it publishes nothing.
fn track(robot: &VirtualRobot, label: &str, estimator: Estimator) -> Result<Tracking, VrError> {
    println!("-- {label} --");

    for _ in 0..SETTLE_SAMPLES {
        publish(robot, &robot.states(), estimator)?;
        robot.rate(HZ);
    }

    let (mut pitch_sum, mut roll_sum, mut rate_sum) = (0.0, 0.0, 0.0);
    let (mut first_alt, mut last_alt) = (0.0, 0.0);
    for i in 0..MEASURE_SAMPLES {
        let s = robot.states();
        let published = publish(robot, &s, estimator)?;

        // There is no attitude on the wire, only a quaternion: these are the
        // angles ex36 extracts, in the frame's own order.
        let [roll, pitch, _yaw] = rotations::quat_to_euler(s.kin.quat, EULER_ORDER);
        // FRD is NED as a world frame, so the third component is DOWN.
        let altitude = -s.kin.lin_pos[2];

        pitch_sum += pitch.to_degrees();
        roll_sum += roll.to_degrees();
        rate_sum += s.kin.ang_vel[2];
        if i == 0 {
            first_alt = altitude;
        }
        last_alt = altitude;

        if i % REPORT_EVERY == 0 {
            // What the loop believes, beside what is true. It is the published
            // pitch the assist drives towards its target, never the real one.
            let believed = match published {
                Some(quat) => format!(
                    "{:+6.2} deg",
                    rotations::quat_to_euler(quat, EULER_ORDER)[1].to_degrees()
                ),
                None => "    --    ".to_string(),
            };
            println!(
                "   t={:7.2}s true pitch={:+6.2} deg  published={believed}  roll={:+6.2} deg  \
                 r={:+7.4} rad/s  alt={:9.1} m",
                s.elapsed,
                pitch.to_degrees(),
                roll.to_degrees(),
                s.kin.ang_vel[2],
                altitude
            );
        }
        robot.rate(HZ);
    }

    let n = f64::from(MEASURE_SAMPLES);
    Ok(Tracking {
        label: label.to_string(),
        mean_pitch_deg: pitch_sum / n,
        mean_roll_deg: roll_sum / n,
        mean_yaw_rate: rate_sum / n,
        altitude_m: last_alt,
        climb_m: last_alt - first_alt,
    })
}

/// One estimate, or none. Returns what went on the wire.
fn publish(
    robot: &VirtualRobot,
    s: &State,
    estimator: Estimator,
) -> Result<Option<[f64; 4]>, VrError> {
    let quat = match estimator {
        Estimator::Silent => return Ok(None),
        Estimator::TruthCopy => s.kin.quat,
        // quat_multiply(a, b) is "b first, then a", so post-multiplying applies
        // the bias about the BODY pitch axis: with the wings level -- which the
        // leveller sees to -- that is exactly PITCH_BIAS_DEG of believed nose-up.
        Estimator::PitchBias => rotations::quat_multiply(s.kin.quat, pitch_bias_quat()),
    };
    // valid = true throughout. The gyro rates go along for the ride; nothing
    // reads them yet.
    robot.publish_estimate(quat, Some(s.sensors.gyroscope.angular_velocity), true)?;
    Ok(Some(quat))
}

/// The invented pitch offset, as a rotation about the body's pitch axis.
///
/// The order is irrelevant for a single-axis rotation and is named anyway: it is
/// the frame's own, and a triple is always stored `[about x, about y, about z]`
/// whatever the application order.
fn pitch_bias_quat() -> [f64; 4] {
    rotations::euler_to_quat([0.0, PITCH_BIAS_DEG.to_radians(), 0.0], EULER_ORDER)
}

/// The scene-authored aircraft's sys_id, from the one command-line argument.
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex35_publish_estimate".to_string());
    match args.next().map(|a| a.parse::<u32>()) {
        Some(Ok(id)) => id,
        _ => {
            eprintln!("usage: {program} <sys_id>\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
