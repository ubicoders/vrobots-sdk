//! ex09 -- state_paced_loop: run once per sample instead of once per tick.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex09_state_paced_loop
//! ```
//!
//! ex01 uses `rate(50.0)`: *your* clock drives the loop, and `states()` hands back
//! whatever the latest snapshot is -- sometimes the same one twice, sometimes
//! skipping one. That is the right default for a controller, which wants to emit
//! an output on a fixed schedule whatever the sensor did.
//!
//! `wait_new_state(timeout)` inverts it: **the data drives the loop.** It blocks
//! until a snapshot newer than the current one arrives, so the body runs exactly
//! once per published sample -- no duplicates, no skips, and no need to guess a
//! rate that divides 25 Hz. Reach for it when you are logging, differentiating,
//! or filtering, where processing a sample twice is a bug.
//!
//! The two things to get right:
//!
//! - **A timeout is not a failure.** `VrError::Timeout` means "no new sample in
//!   time", which is how a paused or stopped sim announces itself; the session is
//!   fine and the next call may well succeed. Handle it and carry on -- do not
//!   `?` it out of `main`, which is what turns a paused sim into a crashed
//!   program. Every other error is real.
//! - **`seq` is the ground truth for drops.** The SDK stores the newest sample it
//!   received; if two arrive between wakeups you see the second one and `seq`
//!   jumps. `stats().seq_gaps` counts that for you.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use std::time::Duration;

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const TIMEOUT: Duration = Duration::from_millis(200); // 5x the 25 Hz period

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    let mut last_seq = 0u64;
    let mut last_t_ns = 0i64;

    // ===== loop =====
    loop {
        match robot.wait_new_state(TIMEOUT) {
            Ok(()) => {
                // Exactly one new sample is waiting -- read it and do the work.
                let s = robot.states();
                let dt_ms = if last_t_ns == 0 {
                    f64::NAN
                } else {
                    (s.t_ns - last_t_ns) as f64 / 1e6
                };
                let skipped = s.seq.saturating_sub(last_seq + 1);
                last_seq = s.seq;
                last_t_ns = s.t_ns;

                let [x, y, z] = s.kin.lin_pos;
                println!(
                    "seq={} dt={dt_ms:6.1} ms pos=({x:.3},{y:.2},{z:.2}){}",
                    s.seq,
                    if skipped > 0 {
                        format!("  <- {skipped} sample(s) skipped")
                    } else {
                        String::new()
                    }
                );
            }
            Err(VrError::Timeout(detail)) => {
                // Not a broken session: no sample arrived in time. The sim is
                // paused, stopped, or the machine is very busy. states() still
                // returns the last snapshot it had.
                let s = robot.states();
                println!(
                    "no new state in {:?} ({detail}); still holding seq={} at t={:.3}",
                    TIMEOUT, s.seq, s.elapsed
                );
            }
            Err(other) => return Err(other), // a real failure
        }
    }
}
