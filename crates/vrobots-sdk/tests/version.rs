//! The version identity, read from the linked C library. No simulator needed.

use vrobots_sdk::{check_version, version, version_info, ConnectOptions, DEFAULT_SRC_ID, VERSION};

#[test]
fn the_linked_library_is_the_release_this_crate_wraps() {
    assert_eq!(VERSION, env!("CARGO_PKG_VERSION"));
    assert_eq!(version(), VERSION);
    check_version().expect("the bundle and the crate come from one release");
}

#[test]
fn version_info_carries_the_build_identity() {
    let v = version_info();
    assert_eq!(v.sdk_version, VERSION);
    assert!(v.schema_version > 0, "schema_version {}", v.schema_version);
    for (name, pin) in [
        ("flatbuffers", &v.flatbuffers),
        ("zenoh", &v.zenoh),
        ("iceoryx2", &v.iceoryx2),
    ] {
        let parts: Vec<&str> = pin.split('.').collect();
        assert!(
            parts.len() >= 3 && parts.iter().all(|p| p.parse::<u32>().is_ok()),
            "{name} pin {pin:?} is not a major.minor.patch version"
        );
    }
    assert!(!v.msgs_commit.is_empty(), "no schema commit recorded");
    assert_eq!(v.src_id, DEFAULT_SRC_ID);
    assert_eq!(ConnectOptions::default().src_id, v.src_id);
}

#[test]
fn the_printed_block_names_every_pin() {
    let v = version_info();
    let text = v.to_string();
    assert!(
        text.starts_with(&format!("vrobots-sdk {VERSION}\n")),
        "{text}"
    );
    for needle in [&v.flatbuffers, &v.zenoh, &v.iceoryx2, &v.msgs_commit] {
        assert!(
            text.contains(needle.as_str()),
            "{needle:?} missing from\n{text}"
        );
    }
    assert!(text.contains(&format!("schema_version {}", v.schema_version)));
    assert!(
        text.ends_with(&format!("src_id        {}", v.src_id)),
        "{text}"
    );
}
