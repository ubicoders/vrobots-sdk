//! Version identity: what this build is, and what it speaks.
//!
//! A schema or IPC version mismatch does not present as an error; it presents
//! as garbage fields or as topics that look absent. So the exact versions are
//! printable from the library a program actually links: [`version_info`] is the
//! block `vrobots --version` prints, and [`check_version`] confirms that the
//! linked C library is the one this crate's bindings describe.

use std::fmt;

use vrobots_sdk_sys as sys;

use crate::error::{VrError, VrResult};
use crate::ffi::{self, fixed_str};

/// The version of this crate, which is also the SDK release it wraps.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// The version of the linked C library, e.g. `"0.1.10"`: the library's own
/// `vrsdk_version()`.
#[must_use]
pub fn version() -> &'static str {
    // SAFETY: `vrsdk_version` takes no arguments and returns a static,
    // NUL-terminated literal.
    unsafe { ffi::static_str(sys::vrsdk_version()) }
}

/// Fail unless the linked C library is the release this crate was built for.
///
/// The bindings are shared with the library by struct layout, so a library
/// from another release reads fields at the wrong offsets with nothing to
/// signal it. The `-sys` crate already refuses a bundle of another version at
/// build time; this is the same check at run time, worth one call at the top of
/// `main` when the shared library is found through `LD_LIBRARY_PATH` or `PATH`.
///
/// # Errors
///
/// [`VrError::Config`] naming both versions when they differ.
pub fn check_version() -> VrResult<()> {
    let linked = version();
    if linked == VERSION {
        Ok(())
    } else {
        Err(VrError::Config(format!(
            "the vrobots-sdk crate is version {VERSION} but the linked vrobots_sdk_capi library \
             is version {linked}. The structs are shared by layout, so mixing releases silently \
             misreads every field"
        )))
    }
}

/// This build's version identity, from [`version_info`].
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct VersionInfo {
    /// The SDK version of the linked library.
    pub sdk_version: String,
    /// `git describe` of the message schema the wire code was built from.
    pub msgs_commit: String,
    /// The `schema_version` this SDK stamps on outbound headers. Compare it with
    /// [`State::schema_version`](crate::State::schema_version).
    pub schema_version: u32,
    /// The flatbuffers version this build was pinned to.
    pub flatbuffers: String,
    /// The zenoh version this build was pinned to.
    pub zenoh: String,
    /// The iceoryx2 version this build was pinned to. iceoryx2 compares
    /// major.minor.patch on every shared-memory open, so a mismatch against the
    /// simulator's presents as "no camera frames", not as an error.
    pub iceoryx2: String,
    /// The `src_id` this build stamps by default.
    pub src_id: u32,
}

impl fmt::Display for VersionInfo {
    /// The block `vrobots --version` prints.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "vrobots-sdk {}", self.sdk_version)?;
        writeln!(
            f,
            "  vrobots_msgs  {} (schema_version {})",
            self.msgs_commit, self.schema_version
        )?;
        writeln!(f, "  flatbuffers   {}", self.flatbuffers)?;
        writeln!(f, "  zenoh         {}", self.zenoh)?;
        writeln!(f, "  iceoryx2      {}", self.iceoryx2)?;
        write!(f, "  src_id        {}", self.src_id)
    }
}

/// This build's version identity: SDK release, schema commit and IPC pins.
///
/// Print it beside any "the simulator is not publishing" report.
///
/// ```
/// let v = vrobots_sdk::version_info();
/// assert_eq!(v.sdk_version, vrobots_sdk::VERSION);
/// println!("{v}");
/// ```
#[must_use]
pub fn version_info() -> VersionInfo {
    let mut raw = sys::vrsdk_version_info_t::default();
    // SAFETY: `raw` is writable storage for one `vrsdk_version_info_t`.
    let code = unsafe { sys::vrsdk_version_info(&mut raw) };
    ffi::expect_ok(code, "vrsdk_version_info");
    VersionInfo {
        sdk_version: fixed_str(&raw.sdk_version),
        msgs_commit: fixed_str(&raw.msgs_commit),
        schema_version: raw.schema_version,
        flatbuffers: fixed_str(&raw.flatbuffers),
        zenoh: fixed_str(&raw.zenoh),
        iceoryx2: fixed_str(&raw.iceoryx2),
        src_id: raw.src_id,
    }
}
