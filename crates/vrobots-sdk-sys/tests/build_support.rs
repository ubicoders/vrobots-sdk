//! Tests of the build script's helpers: asset names, the `SHA256SUMS` parser,
//! verification of a downloaded file, and the marker of a verified download.

// The build script uses items that these tests do not.
#[allow(dead_code)]
#[path = "../build/support.rs"]
mod support;

use std::fs;
use std::path::PathBuf;

use support::{ArchiveKind, Marker, SumsError, VerifyError};

/// SHA-256 of the three bytes `abc` (FIPS 180-2, appendix B.1).
const ABC_SHA256: &str = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
/// SHA-256 of the empty input.
const EMPTY_SHA256: &str = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";

/// A scratch file under Cargo's per-package temporary directory.
fn scratch_file(name: &str, contents: &[u8]) -> PathBuf {
    let path = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name);
    fs::write(&path, contents).unwrap();
    path
}

#[test]
fn asset_names_follow_the_release_layout() {
    let linux = support::bundle_asset("1.2.3", "linux", "x86_64", "gnu").unwrap();
    assert_eq!(linux.file_name, "vrobots_sdk-cpp-1.2.3-linux-x86_64.tar.gz");
    assert_eq!(linux.kind, ArchiveKind::TarGz);

    let windows = support::bundle_asset("1.2.3", "windows", "x86_64", "msvc").unwrap();
    assert_eq!(
        windows.file_name,
        "vrobots_sdk-cpp-1.2.3-windows-x86_64.zip"
    );
    assert_eq!(windows.kind, ArchiveKind::Zip);

    assert!(support::bundle_asset("1.2.3", "linux", "aarch64", "gnu").is_none());
    assert!(support::bundle_asset("1.2.3", "linux", "x86_64", "musl").is_none());
    assert!(support::bundle_asset("1.2.3", "macos", "x86_64", "").is_none());
}

#[test]
fn urls_point_at_the_public_release() {
    assert_eq!(
        support::release_page_url("0.1.10"),
        "https://github.com/ubicoders/vrobots-sdk/releases/tag/v0.1.10"
    );
    let base = support::default_download_base("0.1.10");
    assert_eq!(
        support::asset_url(&base, "SHA256SUMS"),
        "https://github.com/ubicoders/vrobots-sdk/releases/download/v0.1.10/SHA256SUMS"
    );
    assert_eq!(
        support::asset_url("https://example.com/mirror/", "x.zip"),
        "https://example.com/mirror/x.zip"
    );
}

#[test]
fn windows_links_the_import_library_not_the_static_one() {
    let shared = support::library_files("windows", false);
    assert_eq!(shared.link, "vrobots_sdk_capi.dll.lib");
    assert_eq!(shared.runtime, Some("vrobots_sdk_capi.dll"));
    assert_eq!(
        support::library_files("windows", true).link,
        "vrobots_sdk_capi.lib"
    );
    assert_eq!(
        support::library_files("linux", false).link,
        "libvrobots_sdk_capi.so"
    );
    assert_eq!(
        support::library_files("linux", true).link,
        "libvrobots_sdk_capi.a"
    );
    assert_eq!(support::library_files("linux", true).runtime, None);
}

#[test]
fn parses_sha256sum_output_in_text_and_binary_mode() {
    let upper = ABC_SHA256.to_ascii_uppercase();
    let sums = format!(
        "{ABC_SHA256}  vrobots_sdk-cpp-0.1.10-linux-x86_64.tar.gz\r\n\
         \n\
         {EMPTY_SHA256} *vrobots_sdk-cpp-0.1.10-windows-x86_64.zip\n\
         {upper}  ./SHA256SUMS.extra\n"
    );
    let entries = support::parse_sha256sums(&sums);
    assert_eq!(entries.len(), 3);
    assert_eq!(
        entries[0].file_name,
        "vrobots_sdk-cpp-0.1.10-linux-x86_64.tar.gz"
    );
    assert_eq!(entries[0].sha256, ABC_SHA256);
    assert_eq!(
        entries[1].file_name,
        "vrobots_sdk-cpp-0.1.10-windows-x86_64.zip"
    );
    assert_eq!(entries[1].sha256, EMPTY_SHA256);
    // Upper-case digests are normalised and a leading `./` is dropped.
    assert_eq!(entries[2].file_name, "SHA256SUMS.extra");
    assert_eq!(entries[2].sha256, ABC_SHA256);
}

#[test]
fn skips_lines_that_are_not_sha256sum_entries() {
    let short = &ABC_SHA256[..63];
    let sums = format!(
        "# a comment\n\
         {short}  too-short-digest\n\
         {ABC_SHA256}\n\
         {ABC_SHA256} single-space-no-mode\n\
         {ABC_SHA256}  \n\
         SHA256 (bsd-style) = {ABC_SHA256}\n\
         {}  not-hex\n",
        "g".repeat(64)
    );
    assert!(support::parse_sha256sums(&sums).is_empty());
    assert_eq!(
        support::expected_digest(&sums, "x"),
        Err(SumsError::NoEntries)
    );
}

#[test]
fn looks_up_the_digest_of_one_asset() {
    let sums = format!("{ABC_SHA256}  a.tar.gz\n{EMPTY_SHA256}  b.zip\n");
    assert_eq!(
        support::expected_digest(&sums, "b.zip").unwrap(),
        EMPTY_SHA256
    );
    assert_eq!(
        support::expected_digest(&sums, "a.tar.gz").unwrap(),
        ABC_SHA256
    );
}

#[test]
fn reports_a_missing_entry_with_what_is_listed() {
    let sums = format!("{ABC_SHA256}  a.tar.gz\n");
    let err = support::expected_digest(&sums, "b.zip").unwrap_err();
    assert_eq!(
        err,
        SumsError::MissingEntry {
            file_name: "b.zip".to_string(),
            listed: vec!["a.tar.gz".to_string()],
        }
    );
    assert_eq!(
        err.to_string(),
        "SHA256SUMS has no entry for b.zip; it lists a.tar.gz"
    );
}

#[test]
fn rejects_contradictory_entries_but_accepts_repeats() {
    let repeated = format!("{ABC_SHA256}  a.tar.gz\n{ABC_SHA256} *a.tar.gz\n");
    assert_eq!(
        support::expected_digest(&repeated, "a.tar.gz").unwrap(),
        ABC_SHA256
    );

    let conflicting = format!("{ABC_SHA256}  a.tar.gz\n{EMPTY_SHA256}  a.tar.gz\n");
    assert_eq!(
        support::expected_digest(&conflicting, "a.tar.gz"),
        Err(SumsError::Conflicting {
            file_name: "a.tar.gz".to_string()
        })
    );
}

#[test]
fn hashes_known_vectors() {
    assert_eq!(support::sha256_reader(&b"abc"[..]).unwrap(), ABC_SHA256);
    assert_eq!(support::sha256_reader(&b""[..]).unwrap(), EMPTY_SHA256);
    assert_eq!(support::to_hex(&[0x00, 0x0f, 0xa5, 0xff]), "000fa5ff");
}

#[test]
fn verifies_a_file_against_its_listed_digest() {
    let path = scratch_file("verify-abc.bin", b"abc");
    support::verify_file(&path, ABC_SHA256).unwrap();
    support::verify_file(&path, &ABC_SHA256.to_ascii_uppercase()).unwrap();

    match support::verify_file(&path, EMPTY_SHA256) {
        Err(VerifyError::Mismatch { expected, actual }) => {
            assert_eq!(expected, EMPTY_SHA256);
            assert_eq!(actual, ABC_SHA256);
        }
        other => panic!("expected a mismatch, got {other:?}"),
    }
    // A single flipped byte must fail.
    let tampered = scratch_file("verify-abd.bin", b"abd");
    assert!(matches!(
        support::verify_file(&tampered, ABC_SHA256),
        Err(VerifyError::Mismatch { .. })
    ));
    let missing = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("verify-missing.bin");
    assert!(matches!(
        support::verify_file(&missing, ABC_SHA256),
        Err(VerifyError::Io(_))
    ));
}

#[test]
fn marker_round_trips_and_rejects_partial_records() {
    let marker = Marker {
        version: "0.1.10".to_string(),
        asset: "vrobots_sdk-cpp-0.1.10-linux-x86_64.tar.gz".to_string(),
        sha256: ABC_SHA256.to_string(),
    };
    let parsed = Marker::parse(&marker.render()).unwrap();
    assert_eq!(parsed, marker);
    assert!(parsed.is_for("0.1.10", "vrobots_sdk-cpp-0.1.10-linux-x86_64.tar.gz"));
    assert!(!parsed.is_for("0.1.11", "vrobots_sdk-cpp-0.1.10-linux-x86_64.tar.gz"));
    assert!(!parsed.is_for("0.1.10", "vrobots_sdk-cpp-0.1.10-windows-x86_64.zip"));

    assert_eq!(Marker::parse(""), None);
    assert_eq!(Marker::parse("version=0.1.10\nasset=a.tar.gz\n"), None);
    assert_eq!(
        Marker::parse("version=0.1.10\nasset=a.tar.gz\nsha256=abc\n"),
        None
    );
    assert_eq!(
        Marker::parse(&format!("version=\nasset=a.tar.gz\nsha256={ABC_SHA256}\n")),
        None
    );
}

#[test]
fn reads_the_bundle_version_from_its_readme() {
    let readme = "vrobots_sdk C/C++ bundle 0.1.10 (linux-x86_64)\n\n  include/ ...\n";
    assert_eq!(support::readme_bundle_version(readme), Some("0.1.10"));
    assert_eq!(support::readme_bundle_version("something else\n"), None);
    assert_eq!(support::readme_bundle_version(""), None);
}
