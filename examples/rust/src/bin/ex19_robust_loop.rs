//! ex19 -- robust_loop: a loop that survives the simulator going away.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex19_robust_loop
//! ```
//!
//! **Try this while it runs:** stop the simulator (close it, or leave Play mode),
//! wait a few seconds, and start it again. The loop must not exit, must not spin,
//! and must pick the robot back up on its own. That is the whole example.
//!
//! Three behaviours make that work, and each is a deliberate design decision
//! rather than an accident:
//!
//! **1. `states()` never fails and never blocks.** When the sim stops it keeps
//! returning the last snapshot it had -- forever, unchanged. That is the observer
//! contract: a control loop must not raise from a data read. The cost is that a
//! dead sim looks exactly like a stationary robot, so **a stall is not
//! detectable from `states()` alone**. Watch `elapsed` (or `seq`) stop advancing.
//!
//! **2. `wait_new_state()` is the detector.** It returns `VrError::Timeout` when
//! nothing new arrived, which is a *status*, not a fault -- the session is
//! healthy and the next call may succeed. The right handling is to note it, keep
//! the last known state, and try again. Propagating it out of `main` is the bug
//! this example exists to prevent.
//!
//! **3. Nothing tears down.** The zenoh session, the subscriber and the command
//! publisher all outlive the sim's absence. When the simulator returns, samples
//! resume on the same session with no reconnect logic here -- discovery is
//! zenoh's job.
//!
//! The restart is visible in two places, and neither is an error:
//!
//! - **`seq` restarts from 0.** The SDK recognises that as a new publisher
//!   rather than as 7000 lost samples: it logs `state seq went backwards: the
//!   publisher restarted`, and **`seq_gaps` stays where it was.** A gap count
//!   that jumps by thousands after a restart would make the counter useless for
//!   what it is for, which is spotting real drops.
//! - **`elapsed` does not reset.** It is measured from the epoch fixed at this
//!   handle's first sample, so it keeps counting through the outage and comes
//!   back having jumped forward by however long the sim was away. Measured
//!   below: frozen at 5.84 s for the whole outage, then 21.77 s on the first
//!   sample of the new run. If you need the *sim's* run time, take it from the
//!   restarted robot's own `seq`, or reconnect.
//!
//! Commands sent into the void also succeed: publishing to a topic nobody is
//! subscribed to is not an error in zenoh, so `set_mr_pwm` returns `Ok` the whole
//! time the sim is down. There is no reply to a command, ever -- so the echo in
//! the state stream is the only thing that can tell you a robot is listening.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use std::time::{Duration, Instant};

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const TIMEOUT: Duration = Duration::from_millis(500); // ~12x the 25 Hz period
const PWM_US: f64 = 1501.0;
const REPORT_EVERY: u64 = 25; // one status line a second at 25 Hz

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;
    println!("connected. Now stop and restart the simulator -- this loop should ride it out.\n");

    let mut healthy = true;
    let mut samples = 0u64;
    let mut down_since: Option<Instant> = None;

    // ===== loop =====
    loop {
        match robot.wait_new_state(TIMEOUT) {
            Ok(()) => {
                if !healthy {
                    let outage = down_since.map_or(0.0, |t| t.elapsed().as_secs_f64());
                    let stats = robot.stats();
                    let s = robot.states();
                    println!(
                        "RECOVERED after {outage:.1} s -- seq restarted at {} \
                         (elapsed jumped to {:.2}s, it never resets); received={} \
                         seq_gaps={} missed_samples={} -- a restart is not a gap",
                        s.seq, s.elapsed, stats.received, stats.seq_gaps, stats.missed_samples
                    );
                    healthy = true;
                    down_since = None;
                }
                samples += 1;

                let s = robot.states();
                if samples.is_multiple_of(REPORT_EVERY) {
                    let [x, y, z] = s.kin.lin_pos;
                    let stats = robot.stats();
                    println!(
                        "ok  seq={} t={:.2}s pos=({x:.2},{y:.2},{z:.2}) echo={:?} \
                         received={} gaps={} decode_errors={}",
                        s.seq,
                        s.elapsed,
                        s.actuator.pwm,
                        stats.received,
                        stats.seq_gaps,
                        stats.decode_errors
                    );
                }

                // Command as normal while the link is up.
                robot.set_mr_pwm([PWM_US; 4])?;
            }
            Err(VrError::Timeout(_)) => {
                if healthy {
                    println!("\nSTALLED: no new state in {TIMEOUT:?}. Not an error -- holding.");
                    healthy = false;
                    down_since = Some(Instant::now());
                }
                // states() still answers, with the LAST snapshot. Note that
                // `elapsed` is frozen: that, not an exception, is how a dead sim
                // looks from a data read.
                let s = robot.states();
                let down = down_since.map_or(0.0, |t| t.elapsed().as_secs_f64());
                println!(
                    "    down {down:5.1}s -- stale snapshot still readable: seq={} \
                     t={:.2}s (frozen)",
                    s.seq, s.elapsed
                );

                // Publishing into an empty topic is not an error in zenoh, so
                // this keeps returning Ok. A command has no reply; only the echo
                // in the state stream ever proves anything landed.
                robot.set_mr_pwm([PWM_US; 4])?;
            }
            // Anything else is a real failure: the session itself is gone.
            Err(other) => {
                eprintln!("unrecoverable: [{}] {other}", other.code());
                return Err(other);
            }
        }
    }
}
