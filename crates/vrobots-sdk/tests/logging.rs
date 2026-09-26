//! The log bridge, end to end through the C library. One test function,
//! because the handler and the level are process-wide and the test harness
//! runs functions in parallel.

use std::sync::{Arc, Mutex};

use vrobots_sdk::{clear_log_callback, set_log_callback, set_log_level, LogLevel};

type Seen = Arc<Mutex<Vec<(LogLevel, String, String)>>>;

fn recorder(seen: &Seen) -> impl Fn(&vrobots_sdk::LogEvent<'_>) + Send + Sync + 'static {
    let seen = Arc::clone(seen);
    move |event| {
        seen.lock().unwrap().push((
            event.level,
            event.target.to_string(),
            event.message.to_string(),
        ));
    }
}

#[test]
fn events_reach_the_handler_and_the_level_gates_them() {
    let seen: Seen = Arc::default();

    // The library logs its own registration at debug level, which makes a
    // deterministic first event.
    set_log_level(LogLevel::Debug).unwrap();
    set_log_callback(recorder(&seen)).unwrap();
    {
        let events = seen.lock().unwrap();
        assert_eq!(events.len(), 1, "{events:?}");
        let (level, target, message) = &events[0];
        assert_eq!(*level, LogLevel::Debug);
        assert!(!target.is_empty());
        assert!(message.contains("registered"), "{message}");
    }

    // Raising the level silences the same statement at once.
    set_log_level(LogLevel::Off).unwrap();
    set_log_callback(recorder(&seen)).unwrap();
    assert_eq!(seen.lock().unwrap().len(), 1);

    // After unregistering nothing arrives, whatever the level.
    clear_log_callback().unwrap();
    set_log_level(LogLevel::Trace).unwrap();
    assert_eq!(seen.lock().unwrap().len(), 1);

    // A handler that panics is contained: the event is dropped, the program
    // and the library carry on.
    set_log_callback(|_| panic!("a handler bug")).unwrap();
    clear_log_callback().unwrap();

    for (level, name) in [
        (LogLevel::Trace, "trace"),
        (LogLevel::Debug, "debug"),
        (LogLevel::Info, "info"),
        (LogLevel::Warn, "warn"),
        (LogLevel::Error, "error"),
        (LogLevel::Off, "off"),
    ] {
        assert_eq!(level.name(), name);
        assert_eq!(name.parse::<LogLevel>().unwrap(), level);
    }
    assert!(LogLevel::Warn > LogLevel::Info);
}
