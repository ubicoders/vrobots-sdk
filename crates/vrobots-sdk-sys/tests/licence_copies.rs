//! Keeps the licence texts inside the crates equal to the repository's.
//!
//! A published crate carries only its own folder, so every crate of this
//! repository holds copies of `LICENSE` and `LICENSE-ADDENDUM` from the
//! repository root. This test checks every crate under `crates/`, not only this
//! one. Outside the repository, for example in an unpacked package, there is
//! no root to compare with and the test does nothing.

use std::fs;
use std::path::{Path, PathBuf};

/// The licence files every crate ships, named as at the repository root.
const LICENCE_FILES: [&str; 2] = ["LICENSE", "LICENSE-ADDENDUM"];

#[test]
fn every_crate_ships_the_repository_licence_texts() {
    let this_crate = Path::new(env!("CARGO_MANIFEST_DIR"));
    let crates_dir = this_crate.parent().expect("the crate folder has a parent");
    let Some(root) = crates_dir.parent().filter(|root| is_repository_root(root)) else {
        eprintln!(
            "skipped: {} is not inside the repository",
            this_crate.display()
        );
        return;
    };

    let mut crate_dirs: Vec<PathBuf> = fs::read_dir(crates_dir)
        .unwrap_or_else(|err| panic!("cannot list {}: {err}", crates_dir.display()))
        .map(|entry| entry.expect("a readable folder entry").path())
        .filter(|path| path.join("Cargo.toml").is_file())
        .collect();
    crate_dirs.sort();
    assert!(
        crate_dirs.iter().any(|dir| dir == this_crate),
        "listing {} did not find this crate",
        crates_dir.display()
    );

    let mut problems = Vec::new();
    for name in LICENCE_FILES {
        let original = fs::read(root.join(name))
            .unwrap_or_else(|err| panic!("cannot read {}: {err}", root.join(name).display()));
        for dir in &crate_dirs {
            let copy = dir.join(name);
            match fs::read(&copy) {
                Ok(bytes) if bytes == original => {}
                Ok(_) => problems.push(format!("{} differs from the root {name}", copy.display())),
                Err(err) => problems.push(format!("{}: {err}", copy.display())),
            }
        }
    }
    assert!(
        problems.is_empty(),
        "every crate must ship byte-identical copies of the root licence files; copy them from\n\
         {} into each crate folder:\n  {}",
        root.display(),
        problems.join("\n  ")
    );
}

/// Whether `dir` is the root of this repository: its workspace manifest lists
/// this crate, and it holds the licence files.
fn is_repository_root(dir: &Path) -> bool {
    let lists_this_crate = fs::read_to_string(dir.join("Cargo.toml"))
        .is_ok_and(|manifest| manifest.contains("crates/vrobots-sdk-sys"));
    lists_this_crate && LICENCE_FILES.iter().all(|name| dir.join(name).is_file())
}
