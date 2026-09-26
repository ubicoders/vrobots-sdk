//! Unpacking a downloaded bundle archive.
//!
//! Kept apart from `build.rs` so `tests/unpack.rs` can include it and unpack
//! archives laid out like the published ones.

use std::fs;
use std::io;
use std::path::Path;

use crate::support::ArchiveKind;

/// Unpacks `archive` into `destination`, creating the folder if needed.
///
/// Both archive crates refuse entries that would land outside `destination`
/// (absolute paths, `..` components). The published archives have no
/// top-level folder, so `destination` becomes the bundle root.
pub fn unpack(archive: &Path, kind: ArchiveKind, destination: &Path) -> Result<(), String> {
    fs::create_dir_all(destination)
        .map_err(|err| format!("cannot create {}: {err}", destination.display()))?;
    let file = fs::File::open(archive)
        .map_err(|err| format!("cannot open {}: {err}", archive.display()))?;
    match kind {
        ArchiveKind::TarGz => {
            let decoder = flate2::read::GzDecoder::new(io::BufReader::new(file));
            tar::Archive::new(decoder)
                .unpack(destination)
                .map_err(|err| err.to_string())
        }
        ArchiveKind::Zip => zip::ZipArchive::new(io::BufReader::new(file))
            .and_then(|mut zip| zip.extract(destination))
            .map_err(|err| err.to_string()),
    }
}
