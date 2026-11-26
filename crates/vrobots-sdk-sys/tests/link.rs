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
fn nested_structs_agree_with_the_library() {
    // An actuator report built here and decoded there: a layout disagreement in
    // either struct would scramble the four numbers.
    let mut report = sys::vrsdk_actuator_t {
        measured_count: 4,
        ..Default::default()
    };
    report.measured[..4].copy_from_slice(&[2.5, -0.1, 0.448, 0.454]);
    let mut forklift = sys::vrsdk_forklift_actuators_t::default();
    // SAFETY: `report` is an initialised struct and `forklift` is writable.
    let code = unsafe { sys::vrsdk_forklift_actuators_from_actuator(&report, &mut forklift) };
    assert_eq!(code, sys::VRSDK_OK);
    assert_eq!(
        (
            forklift.drive_speed,
            forklift.steering_angle,
            forklift.lift_height,
            forklift.fork_width
        ),
        (2.5, -0.1, 0.448, 0.454)
    );

    // A lidar scan description built here, with its layout nested two levels
    // down, turned into points there: one ring of four beams 90 degrees apart.
    let mut info = sys::vrsdk_lidar_scan_info_t::default();
    info.coord_frame_id[..3].copy_from_slice(&[b'f' as c_char, b'l' as c_char, b'u' as c_char]);
    info.params.rings = 1;
    info.params.columns = 4;
    info.params.azimuth_step = std::f64::consts::FRAC_PI_2;
    info.range_count = 4;
    let ranges = [2.0f32, f32::INFINITY, 1.0, 0.5];
    let mut xyz = [0.0f32; 12];
    // SAFETY: `info` is initialised, `ranges` holds 4 floats and `xyz` is
    // writable for the 12 passed.
    let code =
        unsafe { sys::vrsdk_lidar_points(&info, ranges.as_ptr(), 4, xyz.as_mut_ptr(), xyz.len()) };
    assert_eq!(code, sys::VRSDK_OK);
    assert_eq!(&xyz[..3], &[2.0, 0.0, 0.0], "column 0 is forward");
    assert!(xyz[3..6].iter().all(|v| v.is_nan()), "no return");

    // The spawn pose added to the end of the connect options: the defaults
    // land where the bindings put them.
    let mut options = sys::vrsdk_connect_options_t::default();
    // SAFETY: `options` is writable storage for one struct.
    unsafe { sys::vrsdk_options_default(&mut options) };
    assert!(!options.has_initial_pose);
    assert_eq!(options.initial_orientation, [0.0, 0.0, 0.0, 1.0]);
    assert_eq!(options.src_id, 122);
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
