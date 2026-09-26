//! Helpers shared by the build script and its tests.
//!
//! Nothing in this file touches the network or unpacks an archive, so
//! `tests/build_support.rs` can include it and test it directly. It holds the
//! names of the Release assets and library files, the `SHA256SUMS` parser, the
//! SHA-256 check of a downloaded file, the marker that records a verified
//! download, and the version line of a bundle's `README.txt`.

use std::fmt;
use std::fs::File;
use std::io::{self, Read};
use std::path::Path;

use sha2::{Digest, Sha256};

/// The public repository whose GitHub Releases carry the C bundles.
pub const REPOSITORY_URL: &str = "https://github.com/ubicoders/vrobots-sdk";

/// The file in `OUT_DIR`, beside the unpacked `bundle/`, that records a verified
/// download.
pub const MARKER_FILE: &str = "bundle.verified";

/// The Release page a person opens to download a bundle by hand.
pub fn release_page_url(version: &str) -> String {
    format!("{REPOSITORY_URL}/releases/tag/v{version}")
}

/// The URL prefix under which the assets of the Release `v<version>` are served.
pub fn default_download_base(version: &str) -> String {
    format!("{REPOSITORY_URL}/releases/download/v{version}")
}

/// Joins a URL prefix and a file name with exactly one slash between them.
pub fn asset_url(base: &str, file_name: &str) -> String {
    format!("{}/{}", base.trim_end_matches('/'), file_name)
}

/// How a bundle archive is packed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArchiveKind {
    /// A gzip-compressed tar archive (the Linux bundle).
    TarGz,
    /// A zip archive (the Windows bundle).
    Zip,
}

/// A C bundle published on the Release page.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BundleAsset {
    /// The asset's file name, as `SHA256SUMS` lists it.
    pub file_name: String,
    /// How the archive is packed.
    pub kind: ArchiveKind,
}

/// The published bundle for a target, or `None` when no bundle is published
/// for it.
///
/// `os`, `arch` and `env` are Cargo's `CARGO_CFG_TARGET_OS`,
/// `CARGO_CFG_TARGET_ARCH` and `CARGO_CFG_TARGET_ENV`. The Linux bundle is
/// built against glibc, so a musl target has no published bundle. The Windows
/// bundle is built with MSVC and is offered to every x86_64 Windows target.
pub fn bundle_asset(version: &str, os: &str, arch: &str, env: &str) -> Option<BundleAsset> {
    match (os, arch, env) {
        ("linux", "x86_64", "gnu") => Some(BundleAsset {
            file_name: format!("vrobots_sdk-cpp-{version}-linux-x86_64.tar.gz"),
            kind: ArchiveKind::TarGz,
        }),
        ("windows", "x86_64", _) => Some(BundleAsset {
            file_name: format!("vrobots_sdk-cpp-{version}-windows-x86_64.zip"),
            kind: ArchiveKind::Zip,
        }),
        _ => None,
    }
}

/// The library files a build needs from the bundle's `lib/` folder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LibraryFiles {
    /// The file the linker reads.
    pub link: &'static str,
    /// The file the program loads at run time, or `None` for a static link.
    pub runtime: Option<&'static str>,
}

/// The library files for a target OS and link mode.
///
/// On Windows the bundle carries two `.lib` files: `vrobots_sdk_capi.dll.lib`
/// is the import library of the DLL and `vrobots_sdk_capi.lib` is the static
/// library.
pub fn library_files(os: &str, link_static: bool) -> LibraryFiles {
    match (os, link_static) {
        ("windows", false) => LibraryFiles {
            link: "vrobots_sdk_capi.dll.lib",
            runtime: Some("vrobots_sdk_capi.dll"),
        },
        ("windows", true) => LibraryFiles {
            link: "vrobots_sdk_capi.lib",
            runtime: None,
        },
        ("macos", false) => LibraryFiles {
            link: "libvrobots_sdk_capi.dylib",
            runtime: Some("libvrobots_sdk_capi.dylib"),
        },
        (_, false) => LibraryFiles {
            link: "libvrobots_sdk_capi.so",
            runtime: Some("libvrobots_sdk_capi.so"),
        },
        (_, true) => LibraryFiles {
            link: "libvrobots_sdk_capi.a",
            runtime: None,
        },
    }
}

/// One entry of a `SHA256SUMS` file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SumEntry {
    /// The file name, without the binary-mode `*` and without a leading `./`.
    pub file_name: String,
    /// The digest in lowercase hex.
    pub sha256: String,
}

/// Parses the output of `sha256sum`: one `<64 hex digits> <mode><file name>`
/// entry per line, where the mode character is a space (text mode) or `*`
/// (binary mode).
///
/// Blank lines and lines of any other shape are skipped rather than rejected:
/// only the entry for the asset being verified matters, and
/// [`expected_digest`] reports that entry missing or listed twice with
/// different digests.
pub fn parse_sha256sums(text: &str) -> Vec<SumEntry> {
    text.lines().filter_map(parse_sum_line).collect()
}

fn parse_sum_line(line: &str) -> Option<SumEntry> {
    let line = line.strip_suffix('\r').unwrap_or(line);
    let (digest, rest) = line.split_at_checked(64)?;
    if !digest.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let rest = rest.strip_prefix(' ')?;
    let name = rest.strip_prefix(' ').or_else(|| rest.strip_prefix('*'))?;
    let name = name.strip_prefix("./").unwrap_or(name);
    if name.is_empty() {
        return None;
    }
    Some(SumEntry {
        file_name: name.to_string(),
        sha256: digest.to_ascii_lowercase(),
    })
}

/// Why a `SHA256SUMS` file cannot vouch for an asset.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SumsError {
    /// No line of the file is in `sha256sum` format.
    NoEntries,
    /// The file lists other files, but not this one.
    MissingEntry {
        /// The file that was looked up.
        file_name: String,
        /// The files the file does list.
        listed: Vec<String>,
    },
    /// The file lists this file more than once, with different digests.
    Conflicting {
        /// The file that was looked up.
        file_name: String,
    },
}

impl fmt::Display for SumsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SumsError::NoEntries => {
                write!(f, "SHA256SUMS holds no line in sha256sum format")
            }
            SumsError::MissingEntry { file_name, listed } if listed.is_empty() => {
                write!(f, "SHA256SUMS has no entry for {file_name}")
            }
            SumsError::MissingEntry { file_name, listed } => write!(
                f,
                "SHA256SUMS has no entry for {file_name}; it lists {}",
                listed.join(", ")
            ),
            SumsError::Conflicting { file_name } => write!(
                f,
                "SHA256SUMS lists {file_name} more than once with different digests"
            ),
        }
    }
}

/// The digest, in lowercase hex, that a `SHA256SUMS` file lists for
/// `file_name`.
pub fn expected_digest(sums: &str, file_name: &str) -> Result<String, SumsError> {
    let entries = parse_sha256sums(sums);
    if entries.is_empty() {
        return Err(SumsError::NoEntries);
    }
    let mut found: Option<&str> = None;
    for entry in entries.iter().filter(|entry| entry.file_name == file_name) {
        match found {
            None => found = Some(&entry.sha256),
            Some(previous) if previous == entry.sha256 => {}
            Some(_) => {
                return Err(SumsError::Conflicting {
                    file_name: file_name.to_string(),
                })
            }
        }
    }
    match found {
        Some(digest) => Ok(digest.to_string()),
        None => Err(SumsError::MissingEntry {
            file_name: file_name.to_string(),
            listed: entries.into_iter().map(|entry| entry.file_name).collect(),
        }),
    }
}

/// Lowercase hex of a byte string.
pub fn to_hex(bytes: &[u8]) -> String {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(char::from(DIGITS[usize::from(byte >> 4)]));
        out.push(char::from(DIGITS[usize::from(byte & 0x0f)]));
    }
    out
}

/// The SHA-256 of everything `reader` yields, in lowercase hex.
pub fn sha256_reader(mut reader: impl Read) -> io::Result<String> {
    let mut hasher = Sha256::new();
    let mut buffer = vec![0u8; 256 * 1024];
    loop {
        let read = match reader.read(&mut buffer) {
            Ok(0) => break,
            Ok(read) => read,
            Err(err) if err.kind() == io::ErrorKind::Interrupted => continue,
            Err(err) => return Err(err),
        };
        hasher.update(&buffer[..read]);
    }
    Ok(to_hex(hasher.finalize().as_slice()))
}

/// Why a file failed verification.
#[derive(Debug)]
pub enum VerifyError {
    /// The file could not be read.
    Io(io::Error),
    /// The file's digest differs from the expected one.
    Mismatch {
        /// The digest `SHA256SUMS` lists, in lowercase hex.
        expected: String,
        /// The digest of the file, in lowercase hex.
        actual: String,
    },
}

impl fmt::Display for VerifyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            VerifyError::Io(err) => write!(f, "the downloaded file cannot be read: {err}"),
            VerifyError::Mismatch { expected, actual } => write!(
                f,
                "SHA-256 mismatch: SHA256SUMS lists {expected}, the downloaded file hashes to {actual}"
            ),
        }
    }
}

/// Checks a file against the digest `SHA256SUMS` lists for it.
pub fn verify_file(path: &Path, expected: &str) -> Result<(), VerifyError> {
    let file = File::open(path).map_err(VerifyError::Io)?;
    let actual = sha256_reader(file).map_err(VerifyError::Io)?;
    if actual.eq_ignore_ascii_case(expected) {
        Ok(())
    } else {
        Err(VerifyError::Mismatch {
            expected: expected.to_ascii_lowercase(),
            actual,
        })
    }
}

/// The record of a downloaded and verified bundle, kept in [`MARKER_FILE`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Marker {
    /// The crate version the bundle was downloaded for.
    pub version: String,
    /// The asset file name.
    pub asset: String,
    /// The verified digest of the asset, in lowercase hex.
    pub sha256: String,
}

impl Marker {
    /// The marker as the text written to [`MARKER_FILE`].
    pub fn render(&self) -> String {
        format!(
            "version={}\nasset={}\nsha256={}\n",
            self.version, self.asset, self.sha256
        )
    }

    /// Reads a marker back. Returns `None` for anything that is not a complete
    /// marker, which makes the build download the bundle again.
    pub fn parse(text: &str) -> Option<Marker> {
        let (mut version, mut asset, mut sha256) = (None, None, None);
        for line in text.lines() {
            match line.split_once('=') {
                Some(("version", value)) => version = Some(value),
                Some(("asset", value)) => asset = Some(value),
                Some(("sha256", value)) => sha256 = Some(value),
                _ => {}
            }
        }
        let (version, asset, sha256) = (version?, asset?, sha256?);
        let valid_digest = sha256.len() == 64 && sha256.bytes().all(|b| b.is_ascii_hexdigit());
        if version.is_empty() || asset.is_empty() || !valid_digest {
            return None;
        }
        Some(Marker {
            version: version.to_string(),
            asset: asset.to_string(),
            sha256: sha256.to_ascii_lowercase(),
        })
    }

    /// Whether this marker records the asset a build of `version` needs.
    pub fn is_for(&self, version: &str, asset: &str) -> bool {
        self.version == version && self.asset == asset
    }
}

/// The version a bundle's `README.txt` names on its first line, which reads
/// `vrobots_sdk C/C++ bundle <version> (<platform>)`. `None` when the first
/// line has another shape.
pub fn readme_bundle_version(readme: &str) -> Option<&str> {
    readme
        .lines()
        .next()?
        .trim()
        .strip_prefix("vrobots_sdk C/C++ bundle ")?
        .split_whitespace()
        .next()
}
