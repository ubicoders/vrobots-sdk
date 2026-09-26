//! ex12 -- version_info: what this build speaks, and how well the link is working.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex12_version_info
//! ```
//!
//! Everything here is diagnostics -- the three things to print in a bug report,
//! and the reason each one exists.
//!
//! **1. Version identity.** A schema or IPC version mismatch does not present as
//! an error. It presents as garbage field values, or as topics that look absent.
//! iceoryx2 in particular compares major.minor.patch on every shared-memory open,
//! so a version one patch off **does not error** -- it silently delivers nothing,
//! which reads exactly like "the sim isn't publishing". So the pins have to be
//! printable from the binary you are actually running, not looked up in a file
//! that may not match it.
//!
//! A Rust program has one more version to check: `check_version()` asserts that
//! the `vrobots-sdk` crate and the C library it links are the same release. The
//! structs are shared between them **by layout**, so a library of another
//! release (found through `LD_LIBRARY_PATH` or `PATH`) reads every field at the
//! wrong offset, with nothing to signal it. That is why this program calls it
//! first.
//!
//! **2. Subscriber statistics.** `received`, `decode_errors`, `seq_gaps`,
//! `missed_samples` -- counted continuously, never fatal. A decode error does not
//! tear the session down and does not raise: one malformed payload must not end a
//! flight. `seq_gaps` is the number the sim's own publisher can prove; a growing
//! gap count with a healthy rate means the network (or this process) is dropping
//! samples.
//!
//! **3. `last_error()`.** The error that was counted instead of raised. Empty
//! means nothing has failed to decode; non-empty beside a non-zero
//! `decode_errors` is the actual reason, and it is almost always schema drift
//! between this build and the sim's.
//!
//! The state snapshot also carries the sim's own `schema_version`. Comparing it
//! with ours is the single most useful check in this file.

use std::time::Duration;

use vrobots_sdk::{RobotType, VirtualRobot, VrError, version_info};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const SAMPLES: u32 = 50; // ~2 s at the 25 Hz state rate
const HZ: f64 = 25.0;

fn main() -> Result<(), VrError> {
    vrobots_sdk::init_logging("info");

    // ===== what this build is =====
    vrobots_sdk::check_version()?; // crate and linked library: one release
    let v = version_info();
    println!("{v}"); // the same block `vrobots --version` prints
    println!(
        "  crate         {} (must equal the library above)",
        vrobots_sdk::VERSION
    );

    // ===== what the other end is =====
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;
    let first = robot.states();
    println!(
        "\nsim says: schema_version={} (ours {}), frame={:?} axes={:?}, \
         its header src_id={} (ours {})",
        first.schema_version,
        v.schema_version,
        first.coord_frame_id,
        first.axis_convention.name(),
        first.src_id,
        v.src_id
    );
    if first.schema_version != v.schema_version {
        println!(
            "  MISMATCH -- fields may decode as garbage. Install the SDK release \
             that matches the simulator build."
        );
    }

    // ===== how well it is arriving =====
    println!("\nwatching for {SAMPLES} loop iterations at {HZ} Hz ...");
    let started = std::time::Instant::now();
    for _ in 0..SAMPLES {
        robot.rate(HZ);
    }
    let elapsed = started.elapsed().max(Duration::from_millis(1));

    let stats = robot.stats();
    let last = robot.states();
    println!(
        "\nstats after {:.1} s: received={} decode_errors={} seq_gaps={} \
         missed_samples={} last_seq={}",
        elapsed.as_secs_f64(),
        stats.received,
        stats.decode_errors,
        stats.seq_gaps,
        stats.missed_samples,
        stats.last_seq
    );
    println!(
        "  effective rate {:.1} Hz over the window; sim clock advanced {:.2} s",
        stats.received as f64 / elapsed.as_secs_f64(),
        last.elapsed - first.elapsed
    );

    // Counted, not raised. This is where a decode failure went.
    match robot.last_error() {
        None => println!("  last_error: none -- every payload decoded"),
        Some(e) => println!("  last_error: [{}] {e}", e.code()),
    }
    Ok(())
}
