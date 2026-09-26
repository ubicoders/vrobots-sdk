//! Proves that the bindings link against `vrobots_sdk_capi` and agree with it.
//!
//! None of these calls needs a running simulator. On Linux the build script's
//! rpath lets this test find the shared library; on Windows the DLL must be
//! found through `PATH` when the bundle comes from `VROBOTS_SDK_DIR`.

use std::ffi::{c_char, CStr};
use std::mem::MaybeUninit;

use vrobots_sdk_sys as sys;

/// Reads a NUL-terminated string out of a fixed C `char` array.
fn fixed_str(field: &[c_char]) -> String {
    let bytes: Vec<u8> = field
        .iter()
        .take_while(|&&c| c != 0)
        .map(|&c| c as u8)
        .collect();
    String::from_utf8(bytes).expect("the library writes UTF-8")
}

#[test]
fn library_version_matches_the_crate_version() {
    // SAFETY: `vrsdk_version` takes no arguments and returns a static,
    // NUL-terminated string.
    let pointer = unsafe { sys::vrsdk_version() };
    assert!(!pointer.is_null());
    // SAFETY: checked non-null; the string is static.
    let version = unsafe { CStr::from_ptr(pointer) }.to_str().unwrap();
    assert_eq!(
        version,
        env!("CARGO_PKG_VERSION"),
        "the linked vrobots_sdk_capi reports version {version}; the C bundle must be the one of this crate's version"
    );
}

#[test]
fn version_info_fills_the_struct_the_bindings_declare() {
    let mut info = MaybeUninit::<sys::vrsdk_version_info_t>::zeroed();
    // SAFETY: `info` is writable storage for one `vrsdk_version_info_t`.
    let code = unsafe { sys::vrsdk_version_info(info.as_mut_ptr()) };
    assert_eq!(i64::from(code), i64::from(sys::VRSDK_OK));
    // SAFETY: the call succeeded, so the library wrote the whole struct.
    let info = unsafe { info.assume_init() };

    // A layout disagreement between the bindings and the library would shift
    // these fields and garble them.
    assert_eq!(fixed_str(&info.sdk_version), env!("CARGO_PKG_VERSION"));
    for field in [&info.flatbuffers, &info.zenoh, &info.iceoryx2] {
        let text = fixed_str(field);
        assert!(
            text.starts_with(|c: char| c.is_ascii_digit()),
            "expected a pinned IPC version, got {text:?}"
        );
    }
}

#[test]
fn errors_cross_the_boundary() {
    // SAFETY: a NULL output pointer is rejected with an error code, which is
    // the behaviour under test.
    let code = unsafe { sys::vrsdk_version_info(std::ptr::null_mut()) };
    assert_eq!(i64::from(code), i64::from(sys::VRSDK_ERR_INVALID_ARGUMENT));

    // SAFETY: both functions return static or thread-owned NUL-terminated
    // strings that stay valid until the next SDK call on this thread.
    let (name, detail) = unsafe {
        let name = CStr::from_ptr(sys::vrsdk_error_name(code))
            .to_string_lossy()
            .into_owned();
        let detail = CStr::from_ptr(sys::vrsdk_last_error_message())
            .to_string_lossy()
            .into_owned();
        (name, detail)
    };
    assert!(!name.is_empty() && name != "unknown", "error name {name:?}");
    assert!(
        !detail.is_empty(),
        "the failing call left no detail message"
    );
}
