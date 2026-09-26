//! Helpers every wrapper shares: strings across the boundary, error codes,
//! durations and matrices. Nothing in this module is public.
//!
//! The C API's rules that these helpers rely on, quoted from `vrobots_sdk.h`:
//!
//! * every fallible function returns `vrsdk_err_t`, `VRSDK_OK` (0) or a
//!   `VRSDK_ERR_*` code, and leaves the detail in `vrsdk_last_error_message()`,
//!   valid until the next SDK call **on the same thread**;
//! * out-parameters are written only on success;
//! * strings passed in are borrowed for the duration of the call;
//! * strings passed out are either copied into a caller buffer (always
//!   NUL-terminated, truncated to its capacity) or are static literals.

use std::ffi::{c_char, CStr, CString};
use std::time::Duration;

use vrobots_sdk_sys as sys;

use crate::error::{VrError, VrResult};

/// The longest wait handed to the C API.
///
/// The C layer converts a timeout with `Duration::from_secs_f64`, which cannot
/// represent `Duration::MAX`. A caller who passes `Duration::MAX` means
/// "forever", and 136 years is forever for every practical purpose.
const LONGEST_WAIT: Duration = Duration::from_secs(u32::MAX as u64);

/// A NUL-terminated fixed `char` array from a C struct, as an owned `String`.
///
/// Stops at the first NUL, or at the end of the array if there is none (the
/// library always writes one). Invalid UTF-8, which the library never writes
/// either, is replaced rather than refused.
pub(crate) fn fixed_str(field: &[c_char]) -> String {
    let bytes: Vec<u8> = field
        .iter()
        .take_while(|&&c| c != 0)
        .map(|&c| c.to_ne_bytes()[0])
        .collect();
    match String::from_utf8(bytes) {
        Ok(text) => text,
        Err(e) => String::from_utf8_lossy(e.as_bytes()).into_owned(),
    }
}

/// Write `value` into a fixed `char` array, NUL-terminated.
///
/// A value longer than the array is truncated at a UTF-8 boundary, the same way
/// the library's own writer truncates.
pub(crate) fn write_fixed_str(field: &mut [c_char], value: &str) {
    if field.is_empty() {
        return;
    }
    let mut end = value.len().min(field.len() - 1);
    while end > 0 && !value.is_char_boundary(end) {
        end -= 1;
    }
    for (slot, byte) in field.iter_mut().zip(&value.as_bytes()[..end]) {
        *slot = c_char::from_ne_bytes([*byte]);
    }
    for slot in &mut field[end..] {
        *slot = 0;
    }
}

/// A string the C API returns as a static literal.
///
/// # Safety
///
/// `ptr` must be NULL or point to a NUL-terminated string that lives for the
/// rest of the program. The name functions (`vrsdk_error_name`,
/// `vrsdk_axes_name`, `vrsdk_euler_order_name`, `vrsdk_log_level_name`,
/// `vrsdk_version`) document exactly that: "a static literal -- do not free it".
pub(crate) unsafe fn static_str(ptr: *const c_char) -> &'static str {
    if ptr.is_null() {
        return "";
    }
    // SAFETY: the caller guarantees a NUL-terminated string with static
    // lifetime; the library is linked for the whole run of the program.
    unsafe { CStr::from_ptr(ptr) }.to_str().unwrap_or("")
}

/// The detail string of the most recent failing call on this thread.
///
/// Must be called immediately after the failing call, before any other SDK call
/// on this thread, because the next call overwrites or clears it.
pub(crate) fn last_error_message() -> String {
    // SAFETY: `vrsdk_last_error_message` takes no arguments and never fails.
    let ptr = unsafe { sys::vrsdk_last_error_message() };
    if ptr.is_null() {
        return String::new();
    }
    // SAFETY: the header documents a NUL-terminated string owned by the SDK that
    // stays valid until the next SDK call on this thread. It is copied here,
    // before any other call is made.
    unsafe { CStr::from_ptr(ptr) }
        .to_string_lossy()
        .into_owned()
}

/// Turn a return code into a `Result`, reading the detail message on failure.
pub(crate) fn check(code: sys::vrsdk_err_t) -> VrResult<()> {
    if code == sys::VRSDK_OK {
        Ok(())
    } else {
        Err(VrError::from_last_error(code))
    }
}

/// A call the C API documents as unable to fail with the arguments this crate
/// passes: a live handle, valid pointers, a valid enum value.
///
/// The only way left for it to fail is `VRSDK_ERR_PANIC`, a Rust panic caught at
/// the C boundary, which the header calls "a bug in the SDK, not a condition to
/// handle". It is surfaced here as a panic, the Rust spelling of a bug.
pub(crate) fn expect_ok(code: sys::vrsdk_err_t, what: &str) {
    if let Err(e) = check(code) {
        panic!(
            "{what} failed although the C API documents no failure for these arguments: {e}. \
             This is a bug in the VRobots SDK; please report it."
        );
    }
}

/// A Rust string as a C string, refusing an interior NUL byte by name.
pub(crate) fn c_string(value: &str, what: &str) -> VrResult<CString> {
    CString::new(value).map_err(|_| {
        VrError::InvalidArgument(format!(
            "{what} {value:?} contains a NUL byte, which cannot cross the C boundary"
        ))
    })
}

/// A duration as the seconds the C API takes, capped at [`LONGEST_WAIT`].
pub(crate) fn seconds(duration: Duration) -> f64 {
    duration.min(LONGEST_WAIT).as_secs_f64()
}

/// A row-major 3x3 matrix as the nine flat doubles the C API carries.
pub(crate) fn flatten(m: [[f64; 3]; 3]) -> [f64; 9] {
    [
        m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2],
    ]
}

/// The inverse of [`flatten`].
pub(crate) fn unflatten(f: [f64; 9]) -> [[f64; 3]; 3] {
    [[f[0], f[1], f[2]], [f[3], f[4], f[5]], [f[6], f[7], f[8]]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_strings_stop_at_the_nul_and_survive_a_full_buffer() {
        let mut field: [c_char; 8] = [0; 8];
        write_fixed_str(&mut field, "frd");
        assert_eq!(fixed_str(&field), "frd");

        // Longer than the buffer: truncated, still terminated.
        write_fixed_str(&mut field, "multirotor");
        assert_eq!(fixed_str(&field), "multiro");
        assert_eq!(field[7], 0);

        // A multi-byte character is never cut in half. "ab\u{e9}" is four
        // bytes, so it fits five slots (with the NUL) but not four.
        let mut five: [c_char; 5] = [0; 5];
        write_fixed_str(&mut five, "ab\u{e9}");
        assert_eq!(fixed_str(&five), "ab\u{e9}");
        let mut four: [c_char; 4] = [0; 4];
        write_fixed_str(&mut four, "ab\u{e9}");
        assert_eq!(fixed_str(&four), "ab");
        write_fixed_str(&mut four, "abc\u{e9}");
        assert_eq!(fixed_str(&four), "abc");

        // No NUL at all: the whole array is read.
        let full = [c_char::from_ne_bytes([b'x']); 3];
        assert_eq!(fixed_str(&full), "xxx");
    }

    #[test]
    fn matrices_flatten_row_major_and_back() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        assert_eq!(flatten(m), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        assert_eq!(unflatten(flatten(m)), m);
    }

    #[test]
    fn durations_are_capped_rather_than_overflowing() {
        assert_eq!(seconds(Duration::from_millis(250)), 0.25);
        assert_eq!(seconds(Duration::MAX), f64::from(u32::MAX));
    }

    #[test]
    fn an_interior_nul_is_refused_by_name() {
        let err = c_string("front\0left", "camera name").expect_err("must be refused");
        assert!(matches!(err, VrError::InvalidArgument(_)));
        assert!(err.detail().contains("camera name"), "{err}");
    }
}
