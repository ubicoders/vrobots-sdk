//! ex11 -- topic_discovery: what is on the wire right now, from code.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex11_topic_discovery
//! ```
//!
//! The first thing to run when nothing seems to be happening, and the listing
//! the other examples point you at. No robot, no connect: discovery answers "is
//! the sim publishing at all, and under which ids?" before you have a handle to
//! ask with.
//!
//! The two transports answer that question in completely different ways, and the
//! `observed` flag is where that difference surfaces:
//!
//! - **zenoh has no registry.** Listing means *listening* for `WINDOW`, so a
//!   topic that publishes nothing during the window does not appear at all --
//!   and `hz`, `samples` and `bytes` are real measurements. A short window on a
//!   slow topic is the classic false negative: `vrobots/*/z/frames` publishes at
//!   1 Hz, so a 0.5 s window loses it.
//! - **iceoryx2 has one.** Camera streams come from *reading* it, so they appear
//!   instantly and unmeasured -- all three counters are 0 and `hz` is 0. `live`
//!   is what matters there: `false` marks a stale record whose owning process
//!   died (the table below prints `stale` in the Hz column for those rows).
//!   And because it is shared memory, the iceoryx2 half only ever sees **this
//!   host** -- a remote sim lists states and services but no cameras, which is
//!   not a discovery failure.
//!
//! An empty list is a legitimate answer, not an error: the sim is not in Play
//! mode, is on another machine (pass a router endpoint via `list_topics_with`),
//! or the window was too short for zenoh's discovery.

use std::collections::BTreeMap;
use std::time::Duration;

use vrobots_sdk::{VrError, list_topics};

const WINDOW: Duration = Duration::from_millis(1500);

fn main() -> Result<(), VrError> {
    vrobots_sdk::init_logging("info");

    println!("listening for {WINDOW:?} ...");
    let topics = list_topics(WINDOW)?;

    if topics.is_empty() {
        println!(
            "no vrobots topics. The sim is not in Play mode, is on another host, \
             or {WINDOW:?} was too short for zenoh discovery."
        );
        return Ok(());
    }

    println!("\n{:<4} {:>7} {:>9}  topic", "wire", "Hz", "bytes");
    for t in &topics {
        // `observed` decides whether the numbers mean anything at all.
        let (hz, bytes) = if t.observed {
            (format!("{:.1}", t.hz), t.bytes.to_string())
        } else if t.live {
            ("-".to_string(), "-".to_string())
        } else {
            ("stale".to_string(), "-".to_string())
        };
        println!("[{}] {hz:>7} {bytes:>9}  {}", t.transport.tag(), t.key);
    }

    // The reason to do this in code rather than in the CLI: the result is data.
    // Grouping it by sys_id is how a program answers "which robots exist, and
    // does the one I want have a camera?".
    let mut by_robot: BTreeMap<Option<u32>, Vec<&str>> = BTreeMap::new();
    for t in &topics {
        by_robot.entry(t.sys_id).or_default().push(&t.key);
    }

    println!("\nby robot:");
    for (sys_id, keys) in &by_robot {
        match sys_id {
            Some(id) => println!("  sys_id {id}: {} topic(s)", keys.len()),
            // `manager` and `scene` sit where an id would, so they can never
            // collide with one -- and they parse as None.
            None => println!("  swarm-wide (manager/scene): {} topic(s)", keys.len()),
        }
        for key in keys {
            println!("      {key}");
        }
    }
    Ok(())
}
