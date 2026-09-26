//! ex20 -- logging_tour: where the SDK's diagnostics come from.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex20_logging_tour
//! RUST_LOG=vrobots_sdk=debug cargo run -p vrobots-examples --bin ex20_logging_tour
//! ```
//!
//! The SDK reports through **two channels, and they mean different things**:
//!
//! - **Returned errors** are things *you* did or asked for that could not be
//!   done: a pulse width outside the band, a camera that does not exist, a
//!   service that said no. They are values -- `Result<_, VrError>` -- and they
//!   never appear only in a log.
//! - **Log events** are things the *SDK* did on your behalf: opening a session,
//!   waiting for the first sample, retrying a service, dropping a malformed
//!   payload, reconfiguring a camera that was already mounted (which ends its
//!   old stream). **Nothing in the SDK waits, retries or drops silently**, and
//!   this is where that shows up. Ignore it and a hang has no explanation.
//!
//! The events come from inside the C library, which hands each one to a single
//! registered handler and drops it when there is none. They are not `tracing`
//! events of your program, so a subscriber you install does not see them unless
//! your handler forwards them. Three ways to decide where they go:
//!
//! 1. `vrobots_sdk::init_logging("info")`: the one-liner every example uses. It
//!    registers a handler that prints each event to standard output, in the
//!    shape of `tracing`'s default formatter, and **does nothing if a handler
//!    is already registered**, so it can never fight your own setup.
//! 2. `RUST_LOG`: overrides the argument entirely when set. `RUST_LOG=off`
//!    silences everything, and `vrobots_sdk=debug` is the setting for a hanging
//!    connect. The transports' own events (`zenoh`, `iceoryx2`) are held at
//!    `warn` inside the library, whatever the filter asks for.
//! 3. `vrobots_sdk::set_log_callback`, with a handler of your own registered
//!    before the SDK is used. Skip `init_logging` and forward each `LogEvent`
//!    wherever you like: the `log` or `tracing` crates, JSON, a file. It runs on
//!    SDK threads, so keep it quick, and never call back into the SDK from it.
//!
//! This example turns the volume up, does one noisy thing (connect), one thing
//! that fails as a *value* (an out-of-band pulse width), and one thing that is
//! only ever reported as an event (a state decode failure, via `last_error`).
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const FILTER: &str = "vrobots_sdk=debug,zenoh=warn"; // per-target, like RUST_LOG
const SAMPLES: u32 = 10;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    // The filter syntax is RUST_LOG's, per target. RUST_LOG wins if it is set,
    // which is what makes an example runnable at a different volume without
    // editing it.
    vrobots_sdk::init_logging(FILTER);
    println!("log filter: {FILTER:?} (RUST_LOG overrides it)\n");

    // Calling it twice is harmless: the second call finds a handler already
    // registered and returns. Same reason it is safe to call from a library
    // consumer's main.
    vrobots_sdk::init_logging("error");

    // ===== the noisy moment =====
    // connect() is where a program goes wrong most often, and every wait and
    // retry inside it is an event. This is why examples set up logging BEFORE
    // connecting.
    println!("-- connect: watch the DEBUG lines for the session, the probe and the first sample");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // ===== channel 1: errors are values =====
    println!("\n-- an error is returned, not logged:");
    match robot.set_mr_pwm([0.7; 4]) {
        // 0.7 is a normalised throttle, not a pulse width. The SDK refuses it
        // before publishing rather than clamping, because a clamped 0.7 would
        // look like a valid idle command.
        Ok(()) => println!("   unexpected: 0.7 us was accepted"),
        Err(e) => println!("   [{}] {} -- {}", e.code(), e.kind(), e.detail()),
    }

    // ===== channel 2: events for what the SDK does =====
    println!("\n-- ten states at the sim's own rate (TRACE would show each publish):");
    for _ in 0..SAMPLES {
        robot.wait_new_state(std::time::Duration::from_millis(500))?;
    }
    let s = robot.states();
    println!("   seq={} t={:.3}", s.seq, s.elapsed);

    // ===== the third channel: counted, never raised =====
    // A malformed payload is logged as a warning, counted in stats(), and stored
    // in last_error() -- but never returned from states(), because one bad frame
    // must not end a flight.
    let stats = robot.stats();
    println!(
        "\n-- counted rather than raised: received={} decode_errors={} last_error={}",
        stats.received,
        stats.decode_errors,
        match robot.last_error() {
            Some(e) => format!("[{}] {e}", e.code()),
            None => "none".to_string(),
        }
    );

    println!(
        "\nTry: RUST_LOG=vrobots_sdk=trace to see every publish, or RUST_LOG=off for silence."
    );
    Ok(())
}
