//! Safety-event codes against the library's names, and the swarm-wide stream's
//! lifecycle. That test opens a zenoh session of its own but needs no
//! simulator: it only checks what holds whether or not one is running.

use std::time::{Duration, Instant};

use vrobots_sdk::{
    subscribe_all_safety_events_with, topics, ConnectOptions, SafetyOtherKind, SafetyTier, VrError,
    SAFETY_EVENT_QUEUE_CAPACITY,
};

#[test]
fn tiers_and_kinds_carry_the_names_the_library_gives_them() {
    for (tier, code, name) in [
        (SafetyTier::Unspecified, 0, "unspecified"),
        (SafetyTier::NearMiss, 1, "near_miss"),
        (SafetyTier::Contact, 2, "contact"),
        (SafetyTier::Strike, 3, "strike"),
    ] {
        assert_eq!(tier.code(), code);
        assert_eq!(tier.name(), name);
    }
    for (kind, code, name) in [
        (SafetyOtherKind::Unspecified, 0, "unspecified"),
        (SafetyOtherKind::Person, 1, "person"),
        (SafetyOtherKind::Robot, 2, "robot"),
        (SafetyOtherKind::Object, 3, "object"),
    ] {
        assert_eq!(kind.code(), code);
        assert_eq!(kind.name(), name);
    }
    assert_eq!(SAFETY_EVENT_QUEUE_CAPACITY, 1024);
}

#[test]
fn bad_options_are_refused_before_a_session_opens() {
    let options = ConnectOptions::default().with_client_name("bad\0name");
    let err = subscribe_all_safety_events_with(&options).expect_err("a NUL byte");
    assert!(matches!(err, VrError::InvalidArgument(_)), "{err:?}");
}

#[test]
fn the_swarm_wide_stream_opens_reads_and_stops() {
    let options = ConnectOptions::default().with_connect_timeout(Duration::from_secs(5));
    let events = subscribe_all_safety_events_with(&options).expect("a zenoh session opens");
    assert_eq!(events.key(), topics::ALL_SAFETY_EVENTS);
    assert!(events.is_running());

    // A zero wait is the non-blocking read, never a refusal.
    let started = Instant::now();
    let _ = events.next_timeout(Duration::ZERO);
    let _ = events.next_timeout(Duration::from_millis(50));
    assert!(started.elapsed() < Duration::from_secs(5));

    events.stop();
    events.stop(); // idempotent
    assert!(!events.is_running());
    // Whatever was queued stays readable; once it is drained, a stopped stream
    // answers at once instead of waiting out the timeout.
    let _ = events.drain();
    let started = Instant::now();
    assert!(events.next_timeout(Duration::from_secs(10)).is_none());
    assert!(started.elapsed() < Duration::from_secs(1));
    assert_eq!(events.pending(), 0);
    assert_eq!(events.stats().dropped, 0);
    assert!(events.last_error().is_none() || events.stats().decode_errors > 0);
}
