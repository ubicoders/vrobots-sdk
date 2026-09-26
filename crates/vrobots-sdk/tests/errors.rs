//! The error mapping, for every `VRSDK_ERR_*` code, against the names the C
//! library itself gives them. No simulator needed.

use vrobots_sdk::{err, sys, VrError};

/// A predicate that recognises one variant.
type IsVariant = fn(&VrError) -> bool;

/// Every code the C API defines, its variant, and the library's name for it.
fn table() -> Vec<(i32, IsVariant, &'static str)> {
    vec![
        (
            sys::VRSDK_ERR_SESSION,
            |e| matches!(e, VrError::Session(_)),
            "session",
        ),
        (
            sys::VRSDK_ERR_INVALID_ARGUMENT,
            |e| matches!(e, VrError::InvalidArgument(_)),
            "invalid_argument",
        ),
        (
            sys::VRSDK_ERR_TIMEOUT,
            |e| matches!(e, VrError::Timeout(_)),
            "timeout",
        ),
        (
            sys::VRSDK_ERR_DECODE,
            |e| matches!(e, VrError::Decode(_)),
            "decode",
        ),
        (
            sys::VRSDK_ERR_PUBLISH,
            |e| matches!(e, VrError::Publish(_)),
            "publish",
        ),
        (
            sys::VRSDK_ERR_SERVICE,
            |e| matches!(e, VrError::Service(_)),
            "service",
        ),
        (
            sys::VRSDK_ERR_NO_RESPONDER,
            |e| matches!(e, VrError::NoResponder(_)),
            "no_responder",
        ),
        (
            sys::VRSDK_ERR_DELETED,
            |e| matches!(e, VrError::Deleted(_)),
            "deleted",
        ),
        (
            sys::VRSDK_ERR_CONFIG,
            |e| matches!(e, VrError::Config(_)),
            "config",
        ),
        (
            sys::VRSDK_ERR_PANIC,
            |e| matches!(e, VrError::Panic(_)),
            "panic",
        ),
        (
            sys::VRSDK_ERR_INVALID_HANDLE,
            |e| matches!(e, VrError::InvalidHandle(_)),
            "invalid_handle",
        ),
    ]
}

#[test]
fn every_code_maps_to_its_variant_and_back() {
    for (code, is_variant, name) in table() {
        let e = VrError::from_code(code, "the detail").expect("a failure code");
        assert!(is_variant(&e), "code {code} became {e:?}");
        assert_eq!(e.code(), code);
        assert_eq!(e.kind(), name);
        assert_eq!(err::name(code), name, "the library's own name for {code}");
        assert_eq!(e.detail(), "the detail");
        assert_eq!(e.to_string(), format!("{name}: the detail"));
    }
}

#[test]
fn the_err_constants_are_the_c_codes() {
    assert_eq!(
        [
            err::OK,
            err::SESSION,
            err::INVALID_ARGUMENT,
            err::TIMEOUT,
            err::DECODE,
            err::PUBLISH,
            err::SERVICE,
            err::NO_RESPONDER,
            err::DELETED,
            err::CONFIG,
            err::PANIC,
            err::INVALID_HANDLE,
        ],
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 100, 101]
    );
    assert_eq!(err::name(err::OK), "ok");
}

#[test]
fn success_is_not_an_error_and_unknown_codes_survive() {
    assert!(VrError::from_code(sys::VRSDK_OK, "ignored").is_none());
    VrError::check(sys::VRSDK_OK).expect("OK is success");

    let e = VrError::from_code(4242, "from a newer library").expect("a failure");
    assert!(matches!(e, VrError::Unknown { code: 4242, .. }), "{e:?}");
    assert_eq!(e.code(), 4242);
    assert_eq!(e.kind(), "unknown");
    assert_eq!(e.detail(), "from a newer library");
}

#[test]
fn a_failing_call_carries_the_library_detail() {
    // SAFETY: NULL is refused with an error code and never dereferenced.
    let err = VrError::check(unsafe { sys::vrsdk_version_info(std::ptr::null_mut()) })
        .expect_err("NULL is refused");
    assert!(matches!(err, VrError::InvalidArgument(_)), "{err:?}");
    assert!(!err.detail().is_empty());
    assert_ne!(err.detail(), "no detail available");
}

#[test]
fn a_panic_inside_the_library_is_an_error_not_an_abort() {
    // SAFETY: a test hook with no arguments; the library catches the panic it
    // raises and reports it as `VRSDK_ERR_PANIC`.
    let err = VrError::check(unsafe { sys::vrsdk_panic_for_test() }).expect_err("reported");
    assert!(matches!(err, VrError::Panic(_)), "{err:?}");
    assert!(err.detail().contains("deliberate panic"), "{err}");
}

#[test]
fn the_error_is_a_std_error_and_thread_safe() {
    fn assert_traits<T: std::error::Error + Send + Sync + Clone + 'static>() {}
    assert_traits::<VrError>();
    let boxed: Box<dyn std::error::Error> =
        Box::new(VrError::from_code(err::TIMEOUT, "late").unwrap());
    assert_eq!(boxed.to_string(), "timeout: late");
}
