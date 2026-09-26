//! Logging: the SDK's own events, delivered to your code.
//!
//! Everything the SDK does on your behalf (opening a session, waiting for a
//! first sample, retrying a service, dropping a malformed payload) is an event
//! inside the C library, and nothing waits or drops silently. [`init_logging`]
//! prints those events with a `RUST_LOG`-style filter and is what the examples
//! call; [`set_log_callback`] hands them to a function of yours instead, and
//! [`set_log_level`] sets the lowest level forwarded. Errors are separate: a
//! call that fails returns a [`VrError`](crate::VrError), never only a log line.
//!
//! The events come from the library's own logging, not from the `tracing` crate
//! of your program, so a subscriber you install does not see them unless your
//! callback forwards them. `zenoh` and `iceoryx2` are held at `warn` by the
//! library whatever level is set.

use std::borrow::Cow;
use std::ffi::{c_char, c_void, CStr};
use std::fmt;
use std::io::Write;
use std::panic::{self, AssertUnwindSafe};
use std::str::FromStr;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex, PoisonError, RwLock};
use std::time::{SystemTime, UNIX_EPOCH};

use vrobots_sdk_sys as sys;

use crate::error::VrResult;
use crate::ffi;

/// A log level. Ordered by severity, so `level >= LogLevel::Warn` is a valid
/// test.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LogLevel {
    /// Very fine-grained tracing, including one line per published command.
    Trace,
    /// Diagnostics for when a connect hangs.
    Debug,
    /// Lifecycle: connected, first state sample, camera mounted, robot deleted.
    /// The library's default level.
    Info,
    /// Something was dropped or retried: a decode failure, a sequence gap.
    Warn,
    /// Something failed outright.
    Error,
    /// Not a level of any event: as a threshold it silences everything.
    Off,
}

impl LogLevel {
    /// The level's name, `"trace"` to `"off"`: the library's own
    /// `vrsdk_log_level_name`.
    #[must_use]
    pub fn name(self) -> &'static str {
        // SAFETY: `vrsdk_log_level_name` accepts any value and returns a static,
        // NUL-terminated literal, never NULL.
        unsafe { ffi::static_str(sys::vrsdk_log_level_name(self.code())) }
    }

    fn code(self) -> sys::vrsdk_log_level_t {
        match self {
            LogLevel::Trace => sys::VRSDK_LOG_TRACE,
            LogLevel::Debug => sys::VRSDK_LOG_DEBUG,
            LogLevel::Info => sys::VRSDK_LOG_INFO,
            LogLevel::Warn => sys::VRSDK_LOG_WARN,
            LogLevel::Error => sys::VRSDK_LOG_ERROR,
            LogLevel::Off => sys::VRSDK_LOG_OFF,
        }
    }

    fn from_code(code: sys::vrsdk_log_level_t) -> Option<LogLevel> {
        match code {
            sys::VRSDK_LOG_TRACE => Some(LogLevel::Trace),
            sys::VRSDK_LOG_DEBUG => Some(LogLevel::Debug),
            sys::VRSDK_LOG_INFO => Some(LogLevel::Info),
            sys::VRSDK_LOG_WARN => Some(LogLevel::Warn),
            sys::VRSDK_LOG_ERROR => Some(LogLevel::Error),
            sys::VRSDK_LOG_OFF => Some(LogLevel::Off),
            _ => None,
        }
    }
}

impl fmt::Display for LogLevel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl FromStr for LogLevel {
    type Err = crate::VrError;

    /// Parse a level name, case-insensitively: `"trace"`, `"debug"`, `"info"`,
    /// `"warn"` (or `"warning"`), `"error"` or `"off"`.
    fn from_str(text: &str) -> Result<LogLevel, Self::Err> {
        match text.trim().to_ascii_lowercase().as_str() {
            "trace" => Ok(LogLevel::Trace),
            "debug" => Ok(LogLevel::Debug),
            "info" => Ok(LogLevel::Info),
            "warn" | "warning" => Ok(LogLevel::Warn),
            "error" => Ok(LogLevel::Error),
            "off" => Ok(LogLevel::Off),
            _ => Err(crate::VrError::InvalidArgument(format!(
                "log level {text:?} is not one of trace, debug, info, warn, error, off"
            ))),
        }
    }
}

/// One event from the SDK, as a [`set_log_callback`] handler receives it.
///
/// The strings are borrowed for the duration of the call; copy them to keep
/// them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct LogEvent<'a> {
    /// The event's level. Never [`LogLevel::Off`].
    pub level: LogLevel,
    /// The emitting module, e.g. `"vrobots_sdk::robot"` or `"zenoh"`.
    pub target: &'a str,
    /// The formatted event, including its structured fields.
    pub message: &'a str,
}

type Handler = Arc<dyn Fn(&LogEvent<'_>) + Send + Sync + 'static>;

/// The handler the trampoline calls. Read on SDK threads, written on
/// registration; the read side clones the `Arc` and releases the lock before
/// calling, so a slow handler never blocks a registration.
static HANDLER: RwLock<Option<Handler>> = RwLock::new(None);

/// Serialises registrations, so that "is one installed?" and "install one" in
/// [`init_logging`] are a single step.
static REGISTRATION: Mutex<()> = Mutex::new(());

/// The lowest level the trampoline forwards, as a `VRSDK_LOG_*` code: the last
/// value given to [`set_log_level`], the library's own default until then.
///
/// The library filters too, but it decides once per log statement and caches
/// the answer, so a raised level would not silence a statement that already
/// ran. Checking again here makes raising the level take effect at once.
static MIN_LEVEL: AtomicI32 = AtomicI32::new(sys::VRSDK_LOG_INFO);

/// The one C callback this crate ever registers. It forwards to [`HANDLER`].
///
/// It stays valid for the whole program, which the C API requires of a
/// registered callback, and it never lets a panic from the handler unwind into
/// the library.
unsafe extern "C" fn trampoline(
    level: sys::vrsdk_log_level_t,
    target: *const c_char,
    message: *const c_char,
    _user_data: *mut c_void,
) {
    if level < MIN_LEVEL.load(Ordering::Relaxed) {
        return;
    }
    let _ = panic::catch_unwind(AssertUnwindSafe(|| {
        let handler = HANDLER
            .read()
            .unwrap_or_else(PoisonError::into_inner)
            .clone();
        let Some(handler) = handler else {
            return;
        };
        // SAFETY: the header documents both strings as NUL-terminated and never
        // NULL, borrowed for the duration of this call; `borrowed` also guards
        // against NULL.
        let (target, message) = unsafe { (borrowed(target), borrowed(message)) };
        let event = LogEvent {
            level: LogLevel::from_code(level).unwrap_or(LogLevel::Error),
            target: &target,
            message: &message,
        };
        handler(&event);
    }));
}

/// A borrowed C string as text, without copying when it is valid UTF-8.
///
/// # Safety
///
/// `ptr` must be NULL or a NUL-terminated string valid for `'a`.
unsafe fn borrowed<'a>(ptr: *const c_char) -> Cow<'a, str> {
    if ptr.is_null() {
        return Cow::Borrowed("");
    }
    // SAFETY: per the caller.
    unsafe { CStr::from_ptr(ptr) }.to_string_lossy()
}

/// Register `handler` with the C library, replacing any previous one.
/// The caller holds [`REGISTRATION`].
///
/// The handler is stored before the C registration, because the library logs
/// the registration itself and that event must not reach an empty slot. A
/// refused registration puts the previous handler back.
fn install(handler: Handler) -> VrResult<()> {
    let previous = HANDLER
        .write()
        .unwrap_or_else(PoisonError::into_inner)
        .replace(handler);
    // SAFETY: `trampoline` is a plain function valid for the whole program, safe
    // to call from several threads at once (it only reads an `RwLock`), and it
    // never calls back into the SDK itself: the three requirements
    // `vrsdk_set_log_callback` places on a callback. `user_data` is unused and
    // NULL, which the library passes back without dereferencing.
    let code = unsafe { sys::vrsdk_set_log_callback(Some(trampoline), std::ptr::null_mut()) };
    if let Err(e) = ffi::check(code) {
        *HANDLER.write().unwrap_or_else(PoisonError::into_inner) = previous;
        return Err(e);
    }
    Ok(())
}

/// Route the SDK's log events to `handler`, replacing any handler registered
/// before. Process-wide.
///
/// Register it before [`VirtualRobot::connect`](crate::VirtualRobot::connect):
/// that is the noisiest and most diagnostic moment. The handler is called from
/// SDK threads (the zenoh runtime, each camera reader), possibly concurrently,
/// so it must be quick and thread-safe, and it **must not call back into the
/// SDK**: the library forbids it, and it may deadlock or lose the detail message
/// of the call in progress. A panic inside the handler is caught and the event
/// dropped.
///
/// ```
/// use vrobots_sdk::{LogLevel, set_log_callback, set_log_level};
///
/// set_log_callback(|event| {
///     if event.level >= LogLevel::Warn {
///         eprintln!("[{}] {}: {}", event.level, event.target, event.message);
///     }
/// })?;
/// set_log_level(LogLevel::Debug)?;
/// # vrobots_sdk::clear_log_callback()?;
/// # Ok::<(), vrobots_sdk::VrError>(())
/// ```
///
/// # Errors
///
/// [`VrError::Config`](crate::VrError::Config) if the library cannot take over
/// its own event dispatch in this process.
pub fn set_log_callback<F>(handler: F) -> VrResult<()>
where
    F: Fn(&LogEvent<'_>) + Send + Sync + 'static,
{
    let _registration = REGISTRATION.lock().unwrap_or_else(PoisonError::into_inner);
    install(Arc::new(handler))
}

/// Unregister the log handler, if any. Events are dropped until another is
/// registered.
///
/// # Errors
///
/// As [`set_log_callback`].
pub fn clear_log_callback() -> VrResult<()> {
    let _registration = REGISTRATION.lock().unwrap_or_else(PoisonError::into_inner);
    // SAFETY: `vrsdk_set_log_callback` documents a NULL callback as "unregister"
    // and never dereferences `user_data`, which is NULL here.
    let code = unsafe { sys::vrsdk_set_log_callback(None, std::ptr::null_mut()) };
    ffi::check(code)?;
    *HANDLER.write().unwrap_or_else(PoisonError::into_inner) = None;
    Ok(())
}

/// Set the lowest level forwarded to the handler. The library's default is
/// [`LogLevel::Info`]; [`LogLevel::Off`] silences everything without
/// unregistering.
///
/// Raising the level takes effect at once, on every thread. **Lowering it is
/// best done before [`VirtualRobot::connect`](crate::VirtualRobot::connect)**:
/// the library decides once per log statement whether that statement is
/// enabled, so a statement that already ran while the level was higher stays
/// silent after the level is lowered. [`init_logging`] and a level set before
/// the first connect are unaffected.
///
/// # Errors
///
/// Only if the library reports an internal failure.
pub fn set_log_level(level: LogLevel) -> VrResult<()> {
    // SAFETY: `vrsdk_set_log_level` takes a plain integer and has no pointer
    // arguments; every `LogLevel` maps to one of the codes it accepts.
    let code = unsafe { sys::vrsdk_set_log_level(level.code()) };
    ffi::check(code)?;
    MIN_LEVEL.store(level.code(), Ordering::Relaxed);
    Ok(())
}

/// Print the SDK's events to standard output, filtered like `RUST_LOG`.
///
/// The one line every example starts with. `filter` uses the `RUST_LOG`
/// directive syntax: comma-separated `level` or `target=level` entries, such as
/// `"info"` or `"vrobots_sdk=debug,zenoh=warn"`, where the longest matching
/// target prefix wins. If the `RUST_LOG` environment variable is set, it
/// overrides `filter` entirely, so a program's volume can change without an
/// edit. Lines look like
///
/// ```text
/// 2026-09-25T10:15:30.123456Z  INFO vrobots_sdk::robot: connected
/// ```
///
/// **It does nothing if a handler is already registered**, whether by an
/// earlier call or by [`set_log_callback`], so a second call anywhere in a
/// program cannot fight the first. It returns nothing and cannot fail; if the
/// library refuses the registration, events are simply not printed.
///
/// ```
/// vrobots_sdk::init_logging("info");
/// vrobots_sdk::init_logging("error"); // no effect: the first call won
/// ```
pub fn init_logging(filter: &str) {
    let _registration = REGISTRATION.lock().unwrap_or_else(PoisonError::into_inner);
    if HANDLER
        .read()
        .unwrap_or_else(PoisonError::into_inner)
        .is_some()
    {
        return;
    }

    let from_env = std::env::var("RUST_LOG")
        .ok()
        .map(|text| Filter::parse(&text))
        .filter(Filter::has_directives);
    let filter = from_env.unwrap_or_else(|| Filter::parse(filter));

    if set_log_level(filter.most_verbose()).is_err() {
        return;
    }
    let _ = install(Arc::new(move |event: &LogEvent<'_>| {
        if filter.enabled(event.level, event.target) {
            print_line(event);
        }
    }));
}

/// One event as a line on standard output, in the shape of `tracing`'s default
/// formatter.
fn print_line(event: &LogEvent<'_>) {
    let level = match event.level {
        LogLevel::Trace => "TRACE",
        LogLevel::Debug => "DEBUG",
        LogLevel::Info => "INFO",
        LogLevel::Warn => "WARN",
        LogLevel::Error | LogLevel::Off => "ERROR",
    };
    let line = format!(
        "{} {level:>5} {}: {}\n",
        utc_timestamp(SystemTime::now()),
        event.target,
        event.message
    );
    // A closed or full stdout must not take the program down from an SDK thread.
    let _ = std::io::stdout().lock().write_all(line.as_bytes());
}

/// `SystemTime` as RFC 3339 UTC with microseconds: `2026-09-25T10:15:30.123456Z`.
fn utc_timestamp(time: SystemTime) -> String {
    let since = time.duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = since.as_secs();
    let days = i64::try_from(secs / 86_400).unwrap_or(i64::MAX);
    let rem = secs % 86_400;
    let (year, month, day) = civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{:06}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60,
        since.subsec_micros()
    )
}

/// The proleptic Gregorian date of a day count since 1970-01-01, by Howard
/// Hinnant's `civil_from_days`.
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    (
        year,
        u32::try_from(month).unwrap_or(1),
        u32::try_from(day).unwrap_or(1),
    )
}

/// A `RUST_LOG`-style filter: a default level plus per-target levels.
#[derive(Debug, Clone, PartialEq)]
struct Filter {
    /// The level for targets no directive names, when a bare level was given.
    default: Option<LogLevel>,
    /// `(target prefix, level)`, longest prefix first.
    targets: Vec<(String, LogLevel)>,
}

impl Filter {
    /// Parse directives, skipping the ones this filter cannot express (span
    /// filters, unknown levels), the way `RUST_LOG` parsing is lenient. With no
    /// valid directive at all the filter enables `error` everywhere.
    fn parse(spec: &str) -> Filter {
        let mut default = None;
        let mut targets = Vec::new();
        for directive in spec.split(',').map(str::trim).filter(|d| !d.is_empty()) {
            if directive.contains(['[', '{', '}', ']']) {
                continue;
            }
            match directive.split_once('=') {
                Some((target, level)) => {
                    if let Some(level) = directive_level(level) {
                        let target = target.trim();
                        if target.is_empty() {
                            default = Some(level);
                        } else {
                            targets.push((target.to_string(), level));
                        }
                    }
                }
                None => match directive_level(directive) {
                    Some(level) => default = Some(level),
                    None => targets.push((directive.to_string(), LogLevel::Trace)),
                },
            }
        }
        targets.sort_by_key(|(target, _)| std::cmp::Reverse(target.len()));
        Filter { default, targets }
    }

    fn has_directives(&self) -> bool {
        self.default.is_some() || !self.targets.is_empty()
    }

    /// The threshold for `target`: the longest matching prefix, else the bare
    /// level, else `off` when directives exist and `error` when none do.
    fn threshold(&self, target: &str) -> LogLevel {
        if let Some((_, level)) = self
            .targets
            .iter()
            .find(|(prefix, _)| target.starts_with(prefix.as_str()))
        {
            return *level;
        }
        match self.default {
            Some(level) => level,
            None if self.targets.is_empty() => LogLevel::Error,
            None => LogLevel::Off,
        }
    }

    fn enabled(&self, level: LogLevel, target: &str) -> bool {
        level != LogLevel::Off && level >= self.threshold(target)
    }

    /// The most verbose level any directive asks for: the threshold the
    /// library itself must forward.
    fn most_verbose(&self) -> LogLevel {
        let defaulted = if self.has_directives() {
            self.default
        } else {
            Some(LogLevel::Error)
        };
        self.targets
            .iter()
            .map(|(_, level)| *level)
            .chain(defaulted)
            .min()
            .unwrap_or(LogLevel::Off)
    }
}

/// A directive's level: a name (case-insensitive) or `RUST_LOG`'s numbers,
/// `0` off to `5` trace.
fn directive_level(text: &str) -> Option<LogLevel> {
    match text.trim() {
        "0" => Some(LogLevel::Off),
        "1" => Some(LogLevel::Error),
        "2" => Some(LogLevel::Warn),
        "3" => Some(LogLevel::Info),
        "4" => Some(LogLevel::Debug),
        "5" => Some(LogLevel::Trace),
        other => other.parse().ok(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levels_match_the_library_names_and_order() {
        for (level, name) in [
            (LogLevel::Trace, "trace"),
            (LogLevel::Debug, "debug"),
            (LogLevel::Info, "info"),
            (LogLevel::Warn, "warn"),
            (LogLevel::Error, "error"),
            (LogLevel::Off, "off"),
        ] {
            assert_eq!(level.name(), name);
            assert_eq!(name.parse::<LogLevel>().ok(), Some(level));
            assert_eq!(LogLevel::from_code(level.code()), Some(level));
        }
        assert!(LogLevel::Error > LogLevel::Warn && LogLevel::Warn > LogLevel::Trace);
        assert!("loud".parse::<LogLevel>().is_err());
    }

    #[test]
    fn a_bare_level_applies_everywhere() {
        let f = Filter::parse("info");
        assert!(f.enabled(LogLevel::Info, "vrobots_sdk::robot"));
        assert!(f.enabled(LogLevel::Error, "zenoh"));
        assert!(!f.enabled(LogLevel::Debug, "vrobots_sdk::robot"));
        assert_eq!(f.most_verbose(), LogLevel::Info);
    }

    #[test]
    fn the_longest_target_prefix_wins() {
        let f = Filter::parse("warn,vrobots_sdk=debug,vrobots_sdk::camera=error");
        assert!(f.enabled(LogLevel::Debug, "vrobots_sdk::robot"));
        assert!(!f.enabled(LogLevel::Warn, "vrobots_sdk::camera"));
        assert!(f.enabled(LogLevel::Warn, "zenoh::net"));
        assert!(!f.enabled(LogLevel::Info, "zenoh::net"));
        assert_eq!(f.most_verbose(), LogLevel::Debug);
    }

    #[test]
    fn target_only_directives_disable_the_rest() {
        let f = Filter::parse("vrobots_sdk=debug,zenoh=warn");
        assert!(f.enabled(LogLevel::Debug, "vrobots_sdk::session"));
        assert!(!f.enabled(LogLevel::Error, "iceoryx2"));
        assert!(!f.enabled(LogLevel::Info, "zenoh"));
        // A target with no level means everything from it.
        let f = Filter::parse("vrobots_sdk");
        assert!(f.enabled(LogLevel::Trace, "vrobots_sdk::robot"));
    }

    #[test]
    fn nothing_valid_means_error_and_off_means_silence() {
        let f = Filter::parse("  ,loud,x[span]=info ");
        assert!(f.has_directives(), "a bare target counts as a directive");
        let f = Filter::parse("");
        assert!(!f.has_directives());
        assert!(f.enabled(LogLevel::Error, "anything"));
        assert!(!f.enabled(LogLevel::Warn, "anything"));
        assert_eq!(f.most_verbose(), LogLevel::Error);

        let off = Filter::parse("off");
        assert!(!off.enabled(LogLevel::Error, "vrobots_sdk"));
        assert_eq!(off.most_verbose(), LogLevel::Off);
        assert_eq!(
            Filter::parse("vrobots_sdk=4").most_verbose(),
            LogLevel::Debug
        );
    }

    #[test]
    fn timestamps_are_utc_calendar_dates() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(-1), (1969, 12, 31));
        assert_eq!(civil_from_days(11_016), (2000, 2, 29));
        assert_eq!(civil_from_days(20_721), (2026, 9, 25));
        let t = UNIX_EPOCH + std::time::Duration::new(20_721 * 86_400 + 3_723, 456_789_000);
        assert_eq!(utc_timestamp(t), "2026-09-25T01:02:03.456789Z");
    }
}
