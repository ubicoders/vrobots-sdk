//! ex32 -- fw_rate_controller: your gains, the operator's stick, the sim's aircraft.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex32_fw_rate_controller -- <sys_id>
//! ```
//!
//! ex31 flew the panels open loop. This closes the loop, and it is the shape the
//! fixed wing was built for: **the simulator's rate PIDs bypassed, an external
//! controller in their place, and the operator still flying with the in-game
//! stick.** Three streams meet in one program:
//!
//! ```text
//! z/cmd    ->  SET_ANGVEL from the sim's IMU panel   the setpoint (read, not written)
//! z/state  ->  kin.ang_vel                           the measurement
//! z/cmd    <-  SET_FW_SURFACES + SET_FW_THRUST       your output
//! ```
//!
//! # Commands are readable, because zenoh is a bus
//!
//! Everywhere else in this SDK a command is write-only, and rightly so: it has no
//! reply and the state stream is the proof. `subscribe_setpoint()` is the one
//! exception. The robot's `z/cmd` key is many-to-many, so the setpoints the
//! in-game panel publishes at 50 Hz are readable by anyone who subscribes to the
//! same key -- and a controller that wants the stick as an *input* rather than as
//! a competitor subscribes to it.
//!
//! Everything anyone sends to this robot arrives there, **this process's own
//! traffic included**. [`Setpoint::src_id`] is the sender; compare it against
//! your own [`ConnectOptions::src_id`] and skip your own. Non-matching command
//! ids are counted in [`SetpointStats::filtered`], which climbing fast is normal.
//!
//! # `latest()`, not `fresh()`
//!
//! A setpoint **latches**: the last one stands until the next arrives, and a
//! publisher that stops has not commanded zero. A rate loop therefore wants "the
//! current command" every iteration, which is [`SetpointStream::latest`].
//! [`SetpointStream::fresh`] hands each value out exactly once and is the right
//! read for something that must not act twice on one operator input -- not this.
//! Before the first setpoint ever arrives `latest()` is `None`, and this loop
//! treats that as "hold zero rates", which is a decision, not a default.
//!
//! # Frames, and the one place there are none
//!
//! The setpoint arrives **in the sender's frame, unconverted** -- the panel
//! stamps the target robot's own frame; the Global Hawk publishes `frd`
//! (verified live, sim v3.0.0), so it reads `[p, q, r]` in rad/s. `kin.ang_vel`
//! is in the robot's frame too, so demand and measurement are directly
//! subtractable and the loop below does no conversion at all. It checks the tag
//! rather than assuming.
//!
//! The *output* has no frame: `SET_FW_SURFACES` is a float array, one number per
//! panel, so nothing is re-expressed on the way out and the mixing is entirely
//! yours.
//!
//! # The mixer, since there is not one
//!
//! This reproduces the simulator's own, so the aircraft flies the way it did
//! before you took it over -- panel gains from the airframe:
//!
//! ```text
//! panel 0 = +aileron          panel 3 = 0          (inner flaps: the mixer
//! panel 1 = -aileron          panel 4 = -elevator + rudder   has zero gain
//! panel 2 = 0                 panel 5 = -elevator - rudder   there. ex31.)
//! ```
//!
//! and gains, feed-forward and gain schedule likewise: surface effectiveness
//! grows as airspeed squared, so gains tuned at the 72.8 m/s trim point are far
//! too hot at speed and the loop chatters rail to rail. Scaling the error and the
//! feed-forward by `(V_trim / v)^2` is the same thing as scaling Kp, Ki and FF
//! live.
//!
//! # What this loop is NOT
//!
//! It is a **rate** controller and nothing else. The onboard loop it replaced
//! also carried a wings-level assist, an altitude hold and an airspeed hold; in
//! `DIRECT_SURFACE` none of those exist. With the stick centred the aircraft will
//! hold zero body rates and still wander off in bank and altitude, and the thrust
//! is whatever you last sent. That is the honest cost of taking the airframe.
//!
//! Scene-authored: never deleted, and handed back to its autopilot at the end.
//!
//! [`Setpoint::src_id`]: vrobots_sdk::Setpoint::src_id
//! [`ConnectOptions::src_id`]: vrobots_sdk::ConnectOptions::src_id
//! [`SetpointStats::filtered`]: vrobots_sdk::SetpointStats::filtered
//! [`SetpointStream::latest`]: vrobots_sdk::SetpointStream::latest
//! [`SetpointStream::fresh`]: vrobots_sdk::SetpointStream::fresh

use std::time::Duration;

use vrobots_sdk::{Axes, ConnectOptions, RobotType, VirtualRobot, VrError, cmd};

const USAGE: &str = "\
The Global Hawk is scene-authored (IMU scene) and sys ids are handed out at
scene load and keep incrementing, so there is no id this file could hard-code.
Pass the live one:

    cargo run -p vrobots-examples --bin ex32_fw_rate_controller -- 15

Then fly it with the sim's in-game IMU panel: this loop reads the stick off the
robot's own command topic and closes the loop around it.";

const PANELS: usize = 6;
const LIMIT_RAD: f64 = 20.0_f64 * std::f64::consts::PI / 180.0; // the airframe's clamp
const TRIM_MPS: f64 = 72.8; // the gain schedule's anchor
const CRUISE_N: f64 = 3800.0; // about what the airspeed hold carries at trim

// The simulator's own rate gains, per FRD axis [roll, pitch, yaw].
const KP: [f64; 3] = [0.16, 0.24, 0.35];
const KI: [f64; 3] = [0.05, 0.08, 0.10];
const FF: [f64; 3] = [0.29, 0.09, 0.14];

const RUN_SAMPLES: u32 = 1500; // ~60 s at the 25 Hz state rate
const SAMPLE_TIMEOUT: Duration = Duration::from_millis(500);
const MAX_DT_S: f64 = 0.2; // a stalled stream must not dump seconds into the integrator
const REPORT_EVERY: u32 = 25;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let sys_id = sys_id_from_args();

    // Stamping FRD is documentation: this loop thinks in [p, q, r] and the
    // aircraft publishes in FRD. It sends no vector commands, so nothing is
    // converted either way -- but a header that lies is worse than one that is
    // merely unused.
    let robot = VirtualRobot::connect_with(
        RobotType::GlobalHawk,
        Some(sys_id),
        ConnectOptions::default().with_frame("frd", Axes::FRD),
    )?;
    let own_src_id = robot.options().src_id;

    // Subscribe BEFORE taking the aircraft: a stick input during the handover
    // would otherwise be missed, and the loop would start from "no setpoint".
    let setpoints = robot.subscribe_setpoint()?;
    println!(
        "attached to sys_id={} ({:?}); watching {} for {} (id {}), ignoring src_id={own_src_id}",
        robot.sys_id(),
        robot.robot_type(),
        setpoints.key(),
        cmd::name(setpoints.cmd_id()),
        setpoints.cmd_id()
    );

    let channels = robot.states().actuator.measured.len();
    if channels != PANELS + 1 {
        return Err(VrError::InvalidArgument(format!(
            "this robot publishes {channels} actuator channels; this mixer is written for \
             {PANELS} panels + an engine. Six channels means a simulator too old for the \
             per-panel path."
        )));
    }

    // Mode first, thrust second -- entering direct mode inherits the thrust the
    // airspeed hold was carrying, so a specific thrust has to be asked for after
    // every mode entry (ex31).
    robot.set_fw_ctrl_mode(cmd::FW_DIRECT_SURFACE)?;
    robot.set_fw_thrust(CRUISE_N)?;
    println!("mode -> DIRECT_SURFACE, thrust -> {CRUISE_N} N. Fly it with the sim's IMU panel.\n");

    let mut pid = RatePid::default();
    let mut previous_elapsed = robot.states().elapsed;
    let mut frame_warned = false;

    // ===== loop =====
    for i in 0..RUN_SAMPLES {
        // One control cycle per received state sample, so dt is the state period.
        if let Err(VrError::Timeout(_)) = robot.wait_new_state(SAMPLE_TIMEOUT) {
            // The surfaces latch, so holding is the correct response to silence.
            println!("no new state -- holding the last deflections (they latch)");
            continue;
        }
        let s = robot.states();
        let dt = (s.elapsed - previous_elapsed).clamp(0.0, MAX_DT_S);
        previous_elapsed = s.elapsed;

        // --- the setpoint: latched, so read the current one every iteration ---
        let setpoint = setpoints.latest();
        let demand = match &setpoint {
            // Our own traffic comes back on this bus too. It is not a setpoint.
            Some(sp) if sp.src_id == own_src_id => [0.0; 3],
            Some(sp) => {
                if !frame_warned
                    && !sp.coord_frame_id.is_empty()
                    && sp.coord_frame_id != s.coord_frame_id
                {
                    frame_warned = true;
                    println!(
                        "NOTE: the setpoint is stamped {:?} and this robot reports {:?}. \
                         The vector is NOT converted for you -- convert before subtracting.",
                        sp.coord_frame_id, s.coord_frame_id
                    );
                }
                sp.value
            }
            // Nobody has ever published one. "Hold zero rates" is a decision.
            None => [0.0; 3],
        };

        // --- the measurement: body rates in the robot's own frame ---
        let measured = s.kin.ang_vel;

        // --- gain schedule: surface effectiveness grows as v^2 ---
        let [vx, vy, vz] = s.kin.lin_vel;
        let airspeed = (vx * vx + vy * vy + vz * vz).sqrt().max(1.0);
        let q_scale = (TRIM_MPS * TRIM_MPS / (airspeed * airspeed)).clamp(0.05, 2.0);

        // --- three axes in, three channels out ---
        let [aileron, elevator, rudder] = pid.step(demand, measured, q_scale, dt);

        // --- and the mixer the simulator would have run ---
        let surfaces = mix(aileron, elevator, rudder);
        robot.set_fw_surfaces(&surfaces)?;
        robot.set_fw_thrust(CRUISE_N)?;

        if i % REPORT_EVERY == 0 {
            let stats = setpoints.stats();
            let age = setpoint
                .as_ref()
                .map_or(f64::NAN, |sp| s.elapsed - sp.elapsed);
            println!(
                "t={:7.2}s demand=({:+6.3},{:+6.3},{:+6.3}) measured=({:+6.3},{:+6.3},{:+6.3}) \
                 rad/s  err=({:+6.3},{:+6.3},{:+6.3})",
                s.elapsed,
                demand[0],
                demand[1],
                demand[2],
                measured[0],
                measured[1],
                measured[2],
                demand[0] - measured[0],
                demand[1] - measured[1],
                demand[2] - measured[2],
            );
            println!(
                "        a/e/r=({aileron:+6.3},{elevator:+6.3},{rudder:+6.3}) rad  \
                 panels={:?}  v={airspeed:5.1} m/s q={q_scale:4.2}  setpoints \
                 received={} filtered={} age={age:.2}s",
                surfaces.map(|x| (x * 1000.0).round() / 1000.0),
                stats.received,
                stats.filtered
            );
        }
    }

    // ===== hand it back =====
    robot.set_fw_ctrl_mode(cmd::FW_ONBOARD_RATE)?;
    let stats = setpoints.stats();
    println!(
        "\nmode -> ONBOARD_RATE. setpoints received={} filtered={} decode_errors={} \
         seq_gaps={}. Scene-authored robot: left flying, never deleted.",
        stats.received, stats.filtered, stats.decode_errors, stats.seq_gaps
    );
    Ok(())
}

/// Per-axis rate loop: feed-forward plus PI, with the integrator held at the
/// deflection limit so it cannot wind up behind a saturated surface.
#[derive(Default)]
struct RatePid {
    integral: [f64; 3],
}

impl RatePid {
    /// Returns `[aileron, elevator, rudder]` in radians.
    fn step(&mut self, demand: [f64; 3], measured: [f64; 3], q_scale: f64, dt: f64) -> [f64; 3] {
        let mut out = [0.0; 3];
        for axis in 0..3 {
            let error = q_scale * (demand[axis] - measured[axis]);
            self.integral[axis] += error * dt;

            // Anti-windup: the integral alone can never exceed the clamp, so a
            // stuck surface cannot store minutes of error behind it.
            let integral_limit = if KI[axis] > 0.0 {
                LIMIT_RAD / KI[axis]
            } else {
                0.0
            };
            self.integral[axis] = self.integral[axis].clamp(-integral_limit, integral_limit);

            out[axis] = (FF[axis] * demand[axis] * q_scale
                + KP[axis] * error
                + KI[axis] * self.integral[axis])
                .clamp(-LIMIT_RAD, LIMIT_RAD);
        }
        out
    }
}

/// Three channels onto six panels, with the airframe's own gains.
///
/// The inner flaps stay at zero because that is what the onboard mixer does --
/// not because they cannot move. ex31 moves them, and that is how you know the
/// per-panel path is real.
fn mix(aileron: f64, elevator: f64, rudder: f64) -> [f64; PANELS] {
    [
        aileron,
        -aileron,
        0.0,
        0.0,
        -elevator + rudder,
        -elevator - rudder,
    ]
    .map(|d| d.clamp(-LIMIT_RAD, LIMIT_RAD))
}

/// The scene-authored aircraft's sys_id, from the one command-line argument.
fn sys_id_from_args() -> u32 {
    let mut args = std::env::args();
    let program = args
        .next()
        .unwrap_or_else(|| "ex32_fw_rate_controller".to_string());
    match args.next().map(|a| a.parse::<u32>()) {
        Some(Ok(id)) => id,
        _ => {
            eprintln!("usage: {program} <sys_id>\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
