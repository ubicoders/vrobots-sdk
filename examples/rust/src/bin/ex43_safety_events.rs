//! ex43 -- safety_events: every robot's incidents, one at a time, and the silence between them.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex43_safety_events
//! ```
//!
//! A robot that comes close to a person, touches one or hits one publishes a
//! `SafetyEvent` on its own `vrobots/{id}/z/event/safety`, and an arm's
//! protective stop publishes one when a link meets an object or another robot.
//! This example listens to all of them at once, on `vrobots/*/z/event/safety`,
//! with no robot handle at all: `subscribe_all_safety_events()` opens a zenoh
//! session of its own and closes it when the stream is dropped.
//!
//! # Silence is the normal state
//!
//! Every other stream in the SDK is periodic -- a state at 25 Hz, a scan at 20
//! -- and a quiet one is a broken one. This one is the opposite: **one sample
//! per incident, nothing in between.** A minute without an event is a minute in
//! which nobody got hurt, so `next_timeout()` coming back with nothing is an
//! answer, not an error, and there is no rate to watch.
//!
//! The flip side is that silence cannot tell "nothing happened" from "nobody is
//! publishing". So this example first asks which robots are on the wire at all
//! (ex11's `list_topics`), and refuses to listen to a simulator that is not
//! there.
//!
//! # Events queue
//!
//! A state is superseded by the next one, so the state stream keeps only the
//! latest. An incident is superseded by nothing -- a contact that turns into a
//! strike is two records -- so the stream **queues** every event and hands each
//! one out once, oldest first: `next_timeout()` waits for one, `try_next()` never
//! waits, and `drain()` takes all of them, the read for a control loop. A robot
//! handle has a stream of its own too, `subscribe_safety_events()`.
//!
//! # What an event says
//!
//! | field | meaning |
//! |---|---|
//! | `sys_id`, `robot_name`, `seq` | whose incident it is -- never who was hit -- and that robot's event counter |
//! | `tier` | `near_miss` (close, moving in, never touched), `contact`, `strike` (fast, or a large impulse) |
//! | `other_kind`, `other_name` | `person`, `robot` or `object`, and its name in the scene |
//! | `closing_speed` | the robot's speed toward the other party, m/s, + closing |
//! | `impulse` | N*s; 0 for a near miss, and for an arm link, which is posed, not pushed |
//! | `robot_at_fault` | the robot was the one moving in |
//! | `position` | where, a **world** position in the robot's own frame (`coord_frame_id`) |
//!
//! Only incidents after the subscriber is live arrive; zenoh replays nothing,
//! and a fresh session needs a moment of discovery before the first one can.
//! On the Community tier only the focused robot is on the wire, and another
//! robot's incident is dropped inside the simulator, not queued for later.
//!
//! # Making one happen
//!
//! In the warehouse scene people walk between the racks. Drive the forklift
//! toward one with the keyboard (arrows, focused in the simulator) while this
//! runs, or jog the rail arm into a box stack -- its protective stop reports
//! the link that met it.

use std::time::{Duration, Instant};

use vrobots_sdk::{SafetyEvent, SafetyTier, VrError, list_topics, subscribe_all_safety_events};

const WINDOW: Duration = Duration::from_millis(1500); // ex11's listening window
const LISTEN: Duration = Duration::from_secs(30);
const QUIET_NOTE: Duration = Duration::from_secs(5); // say so after this much silence

fn main() -> Result<(), VrError> {
    // ===== setup: is anybody there? =====
    vrobots_sdk::init_logging("info");
    println!("listening for {WINDOW:?} for the robots on the wire ...");
    let robots = match robots_on_the_wire() {
        Ok(robots) => robots,
        Err(e) => {
            eprintln!("could not look: [{}] {e}", e.code());
            std::process::exit(1);
        }
    };
    if robots.is_empty() {
        eprintln!(
            "no robot is publishing, so no event could ever arrive. Is the simulator running, \
             with a scene in Play mode? \
             `cargo run -p vrobots-examples --bin ex11_topic_discovery` shows what is on the wire."
        );
        std::process::exit(1);
    }
    println!("robots on the wire: {robots:?} -- an event can come from any of them");

    // ===== subscribe, swarm-wide =====
    let events = match subscribe_all_safety_events() {
        Ok(events) => events,
        Err(e) => {
            eprintln!("could not subscribe: [{}] {e}", e.code());
            std::process::exit(1);
        }
    };
    println!("subscribed to {} for {LISTEN:?}\n", events.key());

    // ===== listen =====
    let start = Instant::now();
    let mut near_misses = 0;
    let mut contacts = 0;
    let mut strikes = 0;
    let mut unspecified = 0;
    while start.elapsed() < LISTEN {
        // Wait for the next event, but no longer than QUIET_NOTE, so that a
        // quiet stretch is reported as what it is.
        let wait = QUIET_NOTE.min(LISTEN.saturating_sub(start.elapsed()));
        let Some(event) = events.next_timeout(wait) else {
            println!(
                "t={:5.1}s  quiet: no incident, which is the normal state",
                start.elapsed().as_secs_f64()
            );
            continue;
        };
        print_event(start.elapsed(), &event);
        match event.tier {
            SafetyTier::NearMiss => near_misses += 1,
            SafetyTier::Contact => contacts += 1,
            SafetyTier::Strike => strikes += 1,
            // Unspecified, or a tier newer than this SDK: unknown severity, not harmless.
            _ => unspecified += 1,
        }
    }

    // ===== what the stream counted =====
    // On an aperiodic topic a lost event looks exactly like no event, so these
    // counters are the only way to know the record is complete.
    let stats = events.stats();
    println!(
        "\n{} event(s) in {LISTEN:?}: {near_misses} near miss, {contacts} contact, {strikes} \
         strike, {unspecified} unspecified",
        stats.received
    );
    println!(
        "dropped {} (queue full, nobody reading), missed {} in {} seq gap(s) (lost on the way), \
         decode errors {}, still queued {}",
        stats.dropped,
        stats.missed_events,
        stats.seq_gaps,
        stats.decode_errors,
        events.pending()
    );
    // Dropping the stream undeclares the subscriber and closes its own session.
    Ok(())
}

/// The sys_ids publishing a state topic: the robots on the wire right now.
fn robots_on_the_wire() -> Result<Vec<u32>, VrError> {
    let topics = list_topics(WINDOW)?;
    let mut robots: Vec<u32> = Vec::new();
    for t in &topics {
        let Some(sys_id) = t.sys_id else {
            continue;
        };
        if t.key == vrobots_sdk::topics::state(sys_id) && !robots.contains(&sys_id) {
            robots.push(sys_id);
        }
    }
    Ok(robots)
}

/// One event, as it arrived.
fn print_event(t: Duration, e: &SafetyEvent) {
    println!(
        "t={:5.1}s  robot {} '{}' event #{}: {} with {} '{}'",
        t.as_secs_f64(),
        e.sys_id,
        e.robot_name,
        e.seq,
        e.tier.name(),
        e.other_kind.name(),
        e.other_name
    );
    println!(
        "          closing {:+.2} m/s, impulse {:.1} N*s, robot at fault: {}, at ({:+.2}, {:+.2}, \
         {:+.2}) m in {:?}",
        e.closing_speed,
        e.impulse,
        e.robot_at_fault,
        e.position[0],
        e.position[1],
        e.position[2],
        e.coord_frame_id
    );
}
