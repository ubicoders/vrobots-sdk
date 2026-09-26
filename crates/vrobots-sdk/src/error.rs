//! Errors: one enum for every failure the SDK reports.
//!
//! Every fallible call returns [`VrResult`], and every error is a [`VrError`]
//! that carries two things: a stable numeric [`code`](VrError::code), the same
//! number the C, C++ and Python surfaces and the `vrobots` CLI print, and the
//! SDK's own [`detail`](VrError::detail) sentence naming the topic, service or
//! field involved. Branch on the variant, log the code, show the detail.

use std::fmt;

use vrobots_sdk_sys as sys;

use crate::ffi;

/// Every failure this SDK reports.
///
/// One variant per `VRSDK_ERR_*` code of the C API, each carrying the detail
/// message the library recorded for the failing call. The codes are a stable
/// cross-language contract: a code is never reused or renumbered, and a new
/// failure mode takes the next free number. The enum is `#[non_exhaustive]` so
/// that adding one is not a breaking change here either.
///
/// ```
/// use vrobots_sdk::VrError;
///
/// let e = VrError::from_code(3, "no new state within 200ms (sys_id 1)").unwrap();
/// assert!(matches!(e, VrError::Timeout(_)));
/// assert_eq!(e.code(), 3);
/// assert_eq!(e.kind(), "timeout");
/// assert_eq!(e.to_string(), "timeout: no new state within 200ms (sys_id 1)");
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum VrError {
    /// `VRSDK_ERR_SESSION` (1): the zenoh session could not be opened, or has
    /// gone away.
    Session(String),
    /// `VRSDK_ERR_INVALID_ARGUMENT` (2): bad input from the caller, such as a
    /// pulse width outside its band, an empty camera name or a wrong array
    /// length. Nothing was sent.
    InvalidArgument(String),
    /// `VRSDK_ERR_TIMEOUT` (3): a wait ran out of time, whether for a first
    /// state sample, a service reply or a new snapshot. Never means the session
    /// is broken.
    Timeout(String),
    /// `VRSDK_ERR_DECODE` (4): a payload arrived and did not decode as the
    /// message it should be.
    Decode(String),
    /// `VRSDK_ERR_PUBLISH` (5): publishing to zenoh failed.
    Publish(String),
    /// `VRSDK_ERR_SERVICE` (6): a service answered, and the answer was "no".
    /// Carries the simulator's own reason; see
    /// [`set_skin`](crate::VirtualRobot::set_skin).
    Service(String),
    /// `VRSDK_ERR_NO_RESPONDER` (7): nobody serves that key. Also the capability
    /// probe: asking a truck for `srv/rotors` lands here.
    NoResponder(String),
    /// `VRSDK_ERR_DELETED` (8): the robot was deleted from the simulator and
    /// this handle is spent.
    Deleted(String),
    /// `VRSDK_ERR_CONFIG` (9): the SDK could not be configured as asked, such as
    /// a malformed router endpoint.
    Config(String),
    /// `VRSDK_ERR_PANIC` (100): a panic inside the C library was caught at its
    /// boundary. This is a bug in the SDK, not a condition to handle: the
    /// operation did not happen and the process is intact. Please report it.
    Panic(String),
    /// `VRSDK_ERR_INVALID_HANDLE` (101): a freed or NULL handle reached the C
    /// API. The safe wrapper never passes one, so this too indicates a bug.
    InvalidHandle(String),
    /// A non-zero code this version of the crate does not know, from a newer
    /// C library. Treat it as a failure and read the detail.
    Unknown {
        /// The code the C library returned.
        code: i32,
        /// The detail message the C library recorded.
        detail: String,
    },
}

impl VrError {
    /// The error for a C return code, or `None` for `VRSDK_OK` (0).
    ///
    /// `detail` becomes the variant's message. This is the mapping every call in
    /// this crate uses; it is public so that code calling [`crate::sys`]
    /// directly can build the same errors.
    #[must_use]
    pub fn from_code(code: i32, detail: impl Into<String>) -> Option<VrError> {
        let detail = detail.into();
        let error = match code {
            sys::VRSDK_OK => return None,
            sys::VRSDK_ERR_SESSION => VrError::Session(detail),
            sys::VRSDK_ERR_INVALID_ARGUMENT => VrError::InvalidArgument(detail),
            sys::VRSDK_ERR_TIMEOUT => VrError::Timeout(detail),
            sys::VRSDK_ERR_DECODE => VrError::Decode(detail),
            sys::VRSDK_ERR_PUBLISH => VrError::Publish(detail),
            sys::VRSDK_ERR_SERVICE => VrError::Service(detail),
            sys::VRSDK_ERR_NO_RESPONDER => VrError::NoResponder(detail),
            sys::VRSDK_ERR_DELETED => VrError::Deleted(detail),
            sys::VRSDK_ERR_CONFIG => VrError::Config(detail),
            sys::VRSDK_ERR_PANIC => VrError::Panic(detail),
            sys::VRSDK_ERR_INVALID_HANDLE => VrError::InvalidHandle(detail),
            other => VrError::Unknown {
                code: other,
                detail,
            },
        };
        Some(error)
    }

    /// Turn a return code of a [`crate::sys`] function into a `Result`, reading
    /// the detail message the library recorded for the failing call.
    ///
    /// The C library keeps that message per thread and overwrites it on the
    /// next call, so call this on the same thread, immediately after the call
    /// that returned `code`. This is the check every method of this crate
    /// makes; it is public for code that calls [`crate::sys`] directly.
    ///
    /// ```
    /// use vrobots_sdk::{sys, VrError};
    ///
    /// let mut info = sys::vrsdk_version_info_t::default();
    /// // SAFETY: `info` is writable storage for one `vrsdk_version_info_t`.
    /// VrError::check(unsafe { sys::vrsdk_version_info(&mut info) })?;
    ///
    /// // SAFETY: NULL is refused with an error code, never dereferenced.
    /// let err = VrError::check(unsafe { sys::vrsdk_version_info(std::ptr::null_mut()) })
    ///     .unwrap_err();
    /// assert!(matches!(err, VrError::InvalidArgument(_)));
    /// # Ok::<(), VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// The [`VrError`] for `code` when it is not `VRSDK_OK`.
    pub fn check(code: i32) -> VrResult<()> {
        ffi::check(code)
    }

    /// The error for a failing call, with the detail the library left in its
    /// thread-local last-error slot. `code` must be non-zero.
    pub(crate) fn from_last_error(code: i32) -> VrError {
        let mut detail = ffi::last_error_message();
        if detail.is_empty() {
            detail = "no detail available".to_string();
        }
        VrError::from_code(code, detail).unwrap_or_else(|| VrError::Unknown {
            code,
            detail: "a success code was reported as a failure".to_string(),
        })
    }

    /// The stable numeric code: `1` to `9` from the SDK core, `100` and `101`
    /// from the C layer. Never `0`.
    ///
    /// The same number the C API returns, C++'s `vrsdk::Error::code()`, Python's
    /// `VrError.code` and the CLI's `error [N]` print, and one of the constants
    /// in [`err`].
    #[must_use]
    pub fn code(&self) -> i32 {
        match self {
            VrError::Session(_) => sys::VRSDK_ERR_SESSION,
            VrError::InvalidArgument(_) => sys::VRSDK_ERR_INVALID_ARGUMENT,
            VrError::Timeout(_) => sys::VRSDK_ERR_TIMEOUT,
            VrError::Decode(_) => sys::VRSDK_ERR_DECODE,
            VrError::Publish(_) => sys::VRSDK_ERR_PUBLISH,
            VrError::Service(_) => sys::VRSDK_ERR_SERVICE,
            VrError::NoResponder(_) => sys::VRSDK_ERR_NO_RESPONDER,
            VrError::Deleted(_) => sys::VRSDK_ERR_DELETED,
            VrError::Config(_) => sys::VRSDK_ERR_CONFIG,
            VrError::Panic(_) => sys::VRSDK_ERR_PANIC,
            VrError::InvalidHandle(_) => sys::VRSDK_ERR_INVALID_HANDLE,
            VrError::Unknown { code, .. } => *code,
        }
    }

    /// A short, stable, machine-friendly name for the code: `"timeout"`,
    /// `"invalid_argument"`, `"panic"`, and `"unknown"` for a code this build
    /// does not know. The library's own `vrsdk_error_name`.
    #[must_use]
    pub fn kind(&self) -> &'static str {
        err::name(self.code())
    }

    /// The detail sentence on its own, without the kind prefix that `Display`
    /// adds: `"no new state within 200ms (sys_id 1)"`.
    #[must_use]
    pub fn detail(&self) -> &str {
        match self {
            VrError::Session(s)
            | VrError::InvalidArgument(s)
            | VrError::Timeout(s)
            | VrError::Decode(s)
            | VrError::Publish(s)
            | VrError::Service(s)
            | VrError::NoResponder(s)
            | VrError::Deleted(s)
            | VrError::Config(s)
            | VrError::Panic(s)
            | VrError::InvalidHandle(s) => s,
            VrError::Unknown { detail, .. } => detail,
        }
    }
}

impl fmt::Display for VrError {
    /// `"<kind>: <detail>"`, the line every surface of the SDK prints.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.kind(), self.detail())
    }
}

impl std::error::Error for VrError {}

/// Shorthand for the SDK's fallible operations.
pub type VrResult<T> = Result<T, VrError>;

/// The stable numeric error codes, as plain integers.
///
/// The same values the C API's `VRSDK_ERR_*` constants and Python's `vrsdk.err`
/// class carry, for comparing against [`VrError::code`] or a code logged by
/// another program. Matching on the [`VrError`] variant is the usual way to
/// branch in Rust.
pub mod err {
    use vrobots_sdk_sys as sys;

    use crate::ffi;

    /// Success. Never carried by a [`VrError`](crate::VrError).
    pub const OK: i32 = sys::VRSDK_OK;
    /// See [`VrError::Session`](crate::VrError::Session).
    pub const SESSION: i32 = sys::VRSDK_ERR_SESSION;
    /// See [`VrError::InvalidArgument`](crate::VrError::InvalidArgument).
    pub const INVALID_ARGUMENT: i32 = sys::VRSDK_ERR_INVALID_ARGUMENT;
    /// See [`VrError::Timeout`](crate::VrError::Timeout).
    pub const TIMEOUT: i32 = sys::VRSDK_ERR_TIMEOUT;
    /// See [`VrError::Decode`](crate::VrError::Decode).
    pub const DECODE: i32 = sys::VRSDK_ERR_DECODE;
    /// See [`VrError::Publish`](crate::VrError::Publish).
    pub const PUBLISH: i32 = sys::VRSDK_ERR_PUBLISH;
    /// See [`VrError::Service`](crate::VrError::Service).
    pub const SERVICE: i32 = sys::VRSDK_ERR_SERVICE;
    /// See [`VrError::NoResponder`](crate::VrError::NoResponder).
    pub const NO_RESPONDER: i32 = sys::VRSDK_ERR_NO_RESPONDER;
    /// See [`VrError::Deleted`](crate::VrError::Deleted).
    pub const DELETED: i32 = sys::VRSDK_ERR_DELETED;
    /// See [`VrError::Config`](crate::VrError::Config).
    pub const CONFIG: i32 = sys::VRSDK_ERR_CONFIG;
    /// See [`VrError::Panic`](crate::VrError::Panic).
    pub const PANIC: i32 = sys::VRSDK_ERR_PANIC;
    /// See [`VrError::InvalidHandle`](crate::VrError::InvalidHandle).
    pub const INVALID_HANDLE: i32 = sys::VRSDK_ERR_INVALID_HANDLE;

    /// The short name of a code: `"ok"`, `"timeout"`, `"decode"`, ..., and
    /// `"unknown"` for anything the library does not define.
    #[must_use]
    pub fn name(code: i32) -> &'static str {
        // SAFETY: `vrsdk_error_name` accepts any value and returns a static,
        // NUL-terminated literal ("never NULL; unknown codes give unknown").
        unsafe { ffi::static_str(sys::vrsdk_error_name(code)) }
    }
}
