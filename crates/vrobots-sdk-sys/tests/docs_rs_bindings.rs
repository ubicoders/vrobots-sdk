//! Keeps the documentation copy of the bindings equal to the C bundle's file.
//!
//! docs.rs builds without network access, so with `DOCS_RS` set the build
//! script compiles `bindings/vrobots_sdk.rs`, a copy of the bundle's
//! `bindings.rs` shipped inside this crate, instead of the bundle's own file.
//! A copy that has drifted from the bundle documents an API that real builds do
//! not have. This test compares the two byte for byte in every build against a
//! real bundle, whether it was downloaded or named by `VROBOTS_SDK_DIR`.

use std::fs;
use std::path::Path;

/// The documentation copy, relative to the crate directory.
const DOCS_COPY: &str = "bindings/vrobots_sdk.rs";

/// The file the build script copied to `OUT_DIR/bindings.rs`: the bundle's
/// `bindings.rs`, or the documentation copy itself when `DOCS_RS` is set.
const BINDINGS_SOURCE: &str = env!("VROBOTS_SDK_SYS_BINDINGS_SOURCE");

#[test]
fn documentation_copy_equals_the_bundle_bindings() {
    let copy_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(DOCS_COPY);
    let bundle_path = Path::new(BINDINGS_SOURCE);
    if bundle_path == copy_path {
        // A documentation build compiled the copy itself; there is no bundle
        // file to compare it with.
        eprintln!("skipped: DOCS_RS is set, so this build used {DOCS_COPY} itself");
        return;
    }

    let copy = read(&copy_path);
    let bundle = read(bundle_path);
    if copy != bundle {
        let (line, in_copy, in_bundle) = first_difference(&copy, &bundle);
        panic!(
            "{copy_file} differs from {bundle_file}, the bindings.rs of the C bundle this build\n\
             uses. docs.rs documents the copy and every other build compiles the bundle's file,\n\
             so the two must be equal byte for byte ({copy_len} bytes in the copy, {bundle_len}\n\
             in the bundle's file). First difference, line {line}:\n\
             \x20   copy:   {in_copy}\n\
             \x20   bundle: {in_bundle}\n\
             Replace {DOCS_COPY} with the bundle's bindings.rs, unchanged, and commit it.",
            copy_file = copy_path.display(),
            bundle_file = bundle_path.display(),
            copy_len = copy.len(),
            bundle_len = bundle.len(),
        );
    }
}

fn read(path: &Path) -> Vec<u8> {
    fs::read(path).unwrap_or_else(|err| panic!("cannot read {}: {err}", path.display()))
}

/// The number of the first line where `a` and `b` differ, and that line of
/// each, quoted so that a stray carriage return or trailing space shows.
fn first_difference(a: &[u8], b: &[u8]) -> (usize, String, String) {
    let mut a_lines = a.split(|&byte| byte == b'\n');
    let mut b_lines = b.split(|&byte| byte == b'\n');
    let mut number = 1;
    loop {
        match (a_lines.next(), b_lines.next()) {
            (Some(x), Some(y)) if x == y => number += 1,
            (x, y) => return (number, quote(x), quote(y)),
        }
    }
}

fn quote(line: Option<&[u8]>) -> String {
    match line {
        Some(bytes) => format!("{:?}", String::from_utf8_lossy(bytes)),
        None => "(end of file)".to_string(),
    }
}
