//! Build script of `vrobots-sdk-sys`.
//!
//! It obtains the VRobots SDK C bundle whose version equals this crate's
//! version, copies the bundle's `bindings.rs` to `OUT_DIR/bindings.rs` (which
//! `src/lib.rs` includes), and tells Cargo how to link `vrobots_sdk_capi`.
//!
//! Where the bundle comes from:
//!
//! 1. `VROBOTS_SDK_DIR`, when set and not empty, names an unpacked bundle: a
//!    folder holding `bindings.rs` at its root and the library for the target
//!    in `lib/`. Nothing is downloaded. A relative path is resolved against
//!    this crate's directory, where Cargo runs the script.
//! 2. Otherwise the bundle for the target and `SHA256SUMS` are downloaded from
//!    the Release `v<version>` of the public repository, the bundle is checked
//!    against `SHA256SUMS`, and it is unpacked into `OUT_DIR/bundle`.
//!    `OUT_DIR/bundle.verified` records the version, the asset and its digest,
//!    so a later run of this script reuses the verified copy without network
//!    access. `VROBOTS_SDK_DOWNLOAD_URL` replaces the Release URL prefix, for a
//!    mirror that serves the same files.
//!
//! Bundles are published for x86_64 Linux (glibc) and x86_64 Windows (MSVC).
//! Any other target needs `VROBOTS_SDK_DIR`.
//!
//! Documentation builds: when `DOCS_RS` is set and not empty (docs.rs sets it,
//! and builds without network access), neither of the above happens, even if
//! `VROBOTS_SDK_DIR` is set. The script copies `bindings/vrobots_sdk.rs`, a
//! copy of the bundle's `bindings.rs` shipped inside this crate for
//! documentation builds only, to `OUT_DIR/bindings.rs`, and emits no link
//! directives and no metadata. `cargo doc` and `cargo check` then work offline;
//! anything that calls the library fails to link. `tests/docs_rs_bindings.rs`
//! keeps the copy equal to the bundle's file.
//!
//! Linking: the shared library by default; the static library with the
//! `static` feature. On Linux the script also adds an rpath to the bundle's
//! `lib/` folder; Cargo applies it only to this package's own tests, examples
//! and binaries. Dependents' build scripts receive `DEP_VROBOTS_SDK_CAPI_LIB_DIR`,
//! `DEP_VROBOTS_SDK_CAPI_BUNDLE_DIR` and, when the bundle has headers,
//! `DEP_VROBOTS_SDK_CAPI_INCLUDE`; documentation builds set none of them.
//!
//! This package's own targets are compiled with `VROBOTS_SDK_SYS_BINDINGS_SOURCE`
//! set to the file that became `OUT_DIR/bindings.rs`; the drift test reads it.
//!
//! Every failure ends the build with one message that names what was tried,
//! what failed, and what to do instead, usually building with
//! `VROBOTS_SDK_DIR`.

use std::env;
use std::ffi::OsString;
use std::fmt;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::process;
use std::time::Duration;

#[path = "build/support.rs"]
mod support;
#[path = "build/unpack.rs"]
mod unpack;

use support::{LibraryFiles, Marker};

/// Time allowed to open the connection, including the TLS handshake and any
/// proxy negotiation.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);
/// Time allowed between sending a request and receiving the response headers.
const RESPONSE_TIMEOUT: Duration = Duration::from_secs(60);
/// Time allowed to receive a whole response body. A bundle is some tens of
/// megabytes, so this only ends a transfer that has stalled.
const BODY_TIMEOUT: Duration = Duration::from_secs(30 * 60);
/// `SHA256SUMS` lists a handful of files; a larger response is not one.
const SUMS_SIZE_LIMIT: u64 = 1024 * 1024;
/// The copy of the bundle's `bindings.rs` that ships inside this crate,
/// relative to the crate directory. Only documentation builds read it.
const DOCS_BINDINGS: &str = "bindings/vrobots_sdk.rs";

fn main() {
    if let Err(failure) = run() {
        eprintln!("{failure}");
        process::exit(1);
    }
}

fn run() -> Result<(), Failure> {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=build/support.rs");
    println!("cargo:rerun-if-changed=build/unpack.rs");
    println!("cargo:rerun-if-env-changed=VROBOTS_SDK_DIR");
    println!("cargo:rerun-if-env-changed=VROBOTS_SDK_DOWNLOAD_URL");
    println!("cargo:rerun-if-env-changed=DOCS_RS");

    let build = Build::from_env()?;
    if env::var_os("DOCS_RS").is_some_and(|value| !value.is_empty()) {
        return use_documentation_copy(&build);
    }

    let bundle_dir = match env::var_os("VROBOTS_SDK_DIR").filter(|value| !value.is_empty()) {
        Some(value) => use_local_bundle(&build, value)?,
        None => download_bundle(&build)?,
    };

    let bindings = bundle_dir.join("bindings.rs");
    let destination = build.out_dir.join("bindings.rs");
    fs::copy(&bindings, &destination).map_err(|err| {
        Failure::io(
            &build,
            format!(
                "cannot copy {} to {}",
                bindings.display(),
                destination.display()
            ),
            err,
        )
    })?;
    record_bindings_source(&bindings);

    emit_link_directives(&build, &bundle_dir);
    emit_metadata(&bundle_dir);
    Ok(())
}

/// A documentation build, as on docs.rs: `OUT_DIR/bindings.rs` comes from the
/// copy shipped inside this crate. Nothing is downloaded, `VROBOTS_SDK_DIR` is
/// not read and nothing is linked, so no C bundle is needed.
fn use_documentation_copy(build: &Build) -> Result<(), Failure> {
    println!("cargo:rerun-if-changed={DOCS_BINDINGS}");
    let manifest_dir =
        env::var_os("CARGO_MANIFEST_DIR").ok_or_else(|| not_set("CARGO_MANIFEST_DIR"))?;
    let bindings = Path::new(&manifest_dir).join(DOCS_BINDINGS);
    let destination = build.out_dir.join("bindings.rs");
    fs::copy(&bindings, &destination)
        .map_err(|err| Failure::documentation_copy(&bindings, &destination, err))?;
    record_bindings_source(&bindings);
    println!(
        "cargo:warning=DOCS_RS is set: documentation build using {DOCS_BINDINGS} from this crate; \
         no C bundle is used and vrobots_sdk_capi is not linked, so programs and tests that call \
         it fail to link"
    );
    Ok(())
}

/// Tells this package's own targets which file became `OUT_DIR/bindings.rs`,
/// so `tests/docs_rs_bindings.rs` can compare the documentation copy with the
/// bundle's file.
fn record_bindings_source(bindings: &Path) {
    println!(
        "cargo:rustc-env=VROBOTS_SDK_SYS_BINDINGS_SOURCE={}",
        bindings.display()
    );
}

/// What Cargo tells the script about this build.
struct Build {
    version: String,
    out_dir: PathBuf,
    triple: String,
    os: String,
    arch: String,
    env: String,
    link_static: bool,
}

impl Build {
    fn from_env() -> Result<Build, Failure> {
        Ok(Build {
            version: cargo_env("CARGO_PKG_VERSION")?,
            out_dir: PathBuf::from(env::var_os("OUT_DIR").ok_or_else(|| not_set("OUT_DIR"))?),
            triple: cargo_env("TARGET")?,
            os: cargo_env("CARGO_CFG_TARGET_OS")?,
            arch: cargo_env("CARGO_CFG_TARGET_ARCH")?,
            env: env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default(),
            link_static: env::var_os("CARGO_FEATURE_STATIC").is_some(),
        })
    }

    fn library_files(&self) -> LibraryFiles {
        support::library_files(&self.os, self.link_static)
    }
}

fn cargo_env(name: &str) -> Result<String, Failure> {
    env::var(name).map_err(|_| not_set(name))
}

fn not_set(name: &str) -> Failure {
    Failure {
        summary: format!("the environment variable {name} is not set, or is not UTF-8."),
        facts: Vec::new(),
        advice: "This build script expects to be run by Cargo.\n".to_string(),
    }
}

/// Uses the unpacked bundle `VROBOTS_SDK_DIR` names.
fn use_local_bundle(build: &Build, value: OsString) -> Result<PathBuf, Failure> {
    let given = PathBuf::from(&value);
    let dir = std::path::absolute(&given).unwrap_or(given);
    if !dir.is_dir() {
        return Err(Failure::local_bundle(
            build,
            &dir,
            "the folder does not exist".to_string(),
        ));
    }
    let files = build.library_files();
    check_bundle(&dir, files).map_err(|problem| Failure::local_bundle(build, &dir, problem))?;

    if let Some(runtime) = files.runtime {
        let runtime_path = dir.join("lib").join(runtime);
        if !runtime_path.is_file() {
            println!(
                "cargo:warning=VROBOTS_SDK_DIR has no {}; the build links, but programs will not start until that library can be found",
                runtime_path.display()
            );
        }
    }
    if let Ok(readme) = fs::read_to_string(dir.join("README.txt")) {
        if let Some(found) = support::readme_bundle_version(&readme) {
            if found != build.version {
                println!(
                    "cargo:warning=VROBOTS_SDK_DIR holds the C bundle {found}, but vrobots-sdk-sys is {}; the bindings and the library must come from the bundle of the same version",
                    build.version
                );
            }
        }
    }

    // Rebuild when the bundle is replaced in place, so the copied bindings and
    // the link never go stale.
    println!(
        "cargo:rerun-if-changed={}",
        dir.join("bindings.rs").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        dir.join("lib").join(files.link).display()
    );
    Ok(dir)
}

/// Downloads, verifies and unpacks the bundle into `OUT_DIR/bundle`, or reuses
/// the copy a previous run verified.
fn download_bundle(build: &Build) -> Result<PathBuf, Failure> {
    let asset = support::bundle_asset(&build.version, &build.os, &build.arch, &build.env)
        .ok_or_else(|| Failure::unsupported_target(build))?;
    if build.os == "windows" && build.env != "msvc" {
        println!(
            "cargo:warning=the published Windows C bundle is built with MSVC; linking it into a {} program is not tested",
            build.triple
        );
    }

    let files = build.library_files();
    let bundle_dir = build.out_dir.join("bundle");
    let marker_path = build.out_dir.join(support::MARKER_FILE);
    let previous = fs::read_to_string(&marker_path)
        .ok()
        .and_then(|text| Marker::parse(&text));
    if let Some(marker) = previous {
        if marker.is_for(&build.version, &asset.file_name)
            && check_bundle(&bundle_dir, files).is_ok()
        {
            return Ok(bundle_dir);
        }
    }
    remove_file_if_present(build, &marker_path)?;

    let base = download_base(&build.version);
    let sums_url = support::asset_url(&base, "SHA256SUMS");
    let asset_url = support::asset_url(&base, &asset.file_name);
    let agent = http_agent(&build.version);

    let sums = fetch_text(&agent, &sums_url)
        .map_err(|problem| Failure::download(build, &sums_url, problem))?;
    let expected = support::expected_digest(&sums, &asset.file_name)
        .map_err(|problem| Failure::download(build, &sums_url, problem.to_string()))?;

    let archive = build.out_dir.join(&asset.file_name);
    fetch_file(&agent, &asset_url, &archive)
        .map_err(|problem| Failure::download(build, &asset_url, problem))?;
    if let Err(problem) = support::verify_file(&archive, &expected) {
        // A file that failed verification is never unpacked and never kept.
        let _ = fs::remove_file(&archive);
        return Err(Failure::download(build, &asset_url, problem.to_string()));
    }

    let staging = build.out_dir.join("bundle.partial");
    remove_dir_if_present(build, &staging)?;
    let unpacked = unpack::unpack(&archive, asset.kind, &staging)
        .map_err(|problem| format!("the archive cannot be unpacked: {problem}"))
        .and_then(|()| {
            check_bundle(&staging, files)
                .map_err(|problem| format!("the bundle is incomplete: {problem}"))
        });
    // The archive is not needed once unpacked, and a failed attempt leaves
    // nothing half-written behind.
    let _ = fs::remove_file(&archive);
    if let Err(problem) = unpacked {
        let _ = fs::remove_dir_all(&staging);
        return Err(Failure::download(build, &asset_url, problem));
    }

    remove_dir_if_present(build, &bundle_dir)?;
    fs::rename(&staging, &bundle_dir).map_err(|err| {
        Failure::io(
            build,
            format!(
                "cannot move {} to {}",
                staging.display(),
                bundle_dir.display()
            ),
            err,
        )
    })?;
    let marker = Marker {
        version: build.version.clone(),
        asset: asset.file_name.clone(),
        sha256: expected,
    };
    fs::write(&marker_path, marker.render()).map_err(|err| {
        Failure::io(
            build,
            format!("cannot write {}", marker_path.display()),
            err,
        )
    })?;
    Ok(bundle_dir)
}

/// The URL prefix the assets are fetched from.
fn download_base(version: &str) -> String {
    match env::var("VROBOTS_SDK_DOWNLOAD_URL") {
        Ok(url) if !url.trim().is_empty() => url.trim().trim_end_matches('/').to_string(),
        _ => support::default_download_base(version),
    }
}

/// An HTTP client that uses rustls with the bundled Mozilla root certificates
/// and honours `HTTPS_PROXY`, `HTTP_PROXY`, `ALL_PROXY` and `NO_PROXY`.
fn http_agent(version: &str) -> ureq::Agent {
    ureq::Agent::config_builder()
        .timeout_connect(Some(CONNECT_TIMEOUT))
        .timeout_recv_response(Some(RESPONSE_TIMEOUT))
        .timeout_recv_body(Some(BODY_TIMEOUT))
        .user_agent(format!("vrobots-sdk-sys/{version}"))
        .build()
        .into()
}

fn fetch_text(agent: &ureq::Agent, url: &str) -> Result<String, String> {
    let response = agent.get(url).call().map_err(describe_http_error)?;
    let mut body = response.into_body();
    body.with_config()
        .limit(SUMS_SIZE_LIMIT)
        .read_to_string()
        .map_err(describe_http_error)
}

/// Streams a response body into `destination`, through a temporary name so an
/// interrupted transfer never leaves a complete-looking file.
fn fetch_file(agent: &ureq::Agent, url: &str, destination: &Path) -> Result<(), String> {
    let response = agent.get(url).call().map_err(describe_http_error)?;
    let mut reader = response.into_body().into_reader();
    let partial = destination.with_file_name(format!(
        "{}.part",
        destination
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
    ));
    let mut file = fs::File::create(&partial)
        .map_err(|err| format!("cannot create {}: {err}", partial.display()))?;
    if let Err(err) = io::copy(&mut reader, &mut file) {
        drop(file);
        let _ = fs::remove_file(&partial);
        return Err(format!("the transfer broke off: {err}"));
    }
    drop(file);
    fs::rename(&partial, destination).map_err(|err| {
        format!(
            "cannot move {} to {}: {err}",
            partial.display(),
            destination.display()
        )
    })
}

fn describe_http_error(err: ureq::Error) -> String {
    const NETWORK_HINT: &str = "check network access; behind a proxy, set HTTPS_PROXY";
    match &err {
        ureq::Error::StatusCode(404) => {
            "HTTP 404 Not Found: the Release does not exist, or it does not carry this file"
                .to_string()
        }
        ureq::Error::StatusCode(code) => format!("HTTP status {code}"),
        ureq::Error::HostNotFound => {
            format!("the host name could not be resolved ({NETWORK_HINT})")
        }
        ureq::Error::Io(io) => format!("{io} ({NETWORK_HINT})"),
        ureq::Error::ConnectionFailed | ureq::Error::Timeout(_) => {
            format!("{err} ({NETWORK_HINT})")
        }
        ureq::Error::Tls(_) | ureq::Error::Rustls(_) => format!(
            "TLS failure: {err}. A network that re-signs TLS traffic with its own certificate \
             is not trusted here; use VROBOTS_SDK_DIR"
        ),
        _ => err.to_string(),
    }
}

/// Checks that `dir` holds what this build needs from a bundle.
fn check_bundle(dir: &Path, files: LibraryFiles) -> Result<(), String> {
    if !dir.join("bindings.rs").is_file() {
        return Err("bindings.rs is missing from the bundle root".to_string());
    }
    let link = dir.join("lib").join(files.link);
    if !link.is_file() {
        return Err(format!("lib/{} is missing", files.link));
    }
    Ok(())
}

fn emit_link_directives(build: &Build, bundle_dir: &Path) {
    let lib_dir = bundle_dir.join("lib");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());

    if build.link_static {
        println!("cargo:rustc-link-lib=static=vrobots_sdk_capi");
        return;
    }
    if build.os == "windows" {
        // rustc turns `dylib=vrobots_sdk_capi` into `vrobots_sdk_capi.lib`,
        // which in the bundle is the static library. The DLL's import library
        // is `vrobots_sdk_capi.dll.lib`; `+verbatim` makes rustc pass exactly
        // that file name to the linker.
        println!("cargo:rustc-link-lib=dylib:+verbatim=vrobots_sdk_capi.dll.lib");
    } else {
        println!("cargo:rustc-link-lib=dylib=vrobots_sdk_capi");
    }

    if build.os == "linux" {
        let lib_dir = lib_dir.display().to_string();
        if lib_dir.contains(',') {
            // `-Wl,` splits its argument at commas, which would cut the path.
            println!(
                "cargo:warning=no rpath added: the path {lib_dir} contains a comma; set LD_LIBRARY_PATH to run this package's tests"
            );
        } else {
            println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
        }
    }
}

fn emit_metadata(bundle_dir: &Path) {
    println!("cargo:bundle_dir={}", bundle_dir.display());
    println!("cargo:lib_dir={}", bundle_dir.join("lib").display());
    let include = bundle_dir.join("include");
    if include.is_dir() {
        println!("cargo:include={}", include.display());
    }
}

fn remove_file_if_present(build: &Build, path: &Path) -> Result<(), Failure> {
    match fs::remove_file(path) {
        Err(err) if err.kind() != io::ErrorKind::NotFound => Err(Failure::io(
            build,
            format!("cannot remove {}", path.display()),
            err,
        )),
        _ => Ok(()),
    }
}

fn remove_dir_if_present(build: &Build, path: &Path) -> Result<(), Failure> {
    match fs::remove_dir_all(path) {
        Err(err) if err.kind() != io::ErrorKind::NotFound => Err(Failure::io(
            build,
            format!("cannot remove {}", path.display()),
            err,
        )),
        _ => Ok(()),
    }
}

/// A problem that ends the build, printed as one message.
struct Failure {
    /// One sentence: what could not be done.
    summary: String,
    /// Labelled facts: the URL or folder involved and what went wrong.
    facts: Vec<(&'static str, String)>,
    /// What to do about it.
    advice: String,
}

impl Failure {
    fn download(build: &Build, url: &str, problem: String) -> Failure {
        Failure {
            summary: format!(
                "cannot download the VRobots SDK C bundle {}.",
                build.version
            ),
            facts: vec![("tried", url.to_string()), ("failed", problem)],
            advice: override_advice(build),
        }
    }

    fn unsupported_target(build: &Build) -> Failure {
        Failure {
            summary: format!("no VRobots SDK C bundle is published for the target {}.", build.triple),
            facts: vec![(
                "published",
                "x86_64 Linux with glibc (x86_64-unknown-linux-gnu) and x86_64 Windows (x86_64-pc-windows-msvc)"
                    .to_string(),
            )],
            advice: override_advice(build),
        }
    }

    fn local_bundle(build: &Build, dir: &Path, problem: String) -> Failure {
        Failure {
            summary: "VROBOTS_SDK_DIR does not name a usable VRobots SDK C bundle.".to_string(),
            facts: vec![("folder", dir.display().to_string()), ("failed", problem)],
            advice: format!(
                "VROBOTS_SDK_DIR must name an unpacked C bundle: bindings.rs at its root and\n\
                 lib/{} inside it. Download the bundle {} for your platform from\n    {}\n\
                 or unset VROBOTS_SDK_DIR to let the build download it.\n",
                build.library_files().link,
                build.version,
                support::release_page_url(&build.version)
            ),
        }
    }

    fn io(build: &Build, what: String, err: io::Error) -> Failure {
        Failure {
            summary: "cannot prepare the VRobots SDK C bundle in the build directory.".to_string(),
            facts: vec![("failed", format!("{what}: {err}"))],
            advice: override_advice(build),
        }
    }

    fn documentation_copy(bindings: &Path, destination: &Path, err: io::Error) -> Failure {
        Failure {
            summary: "cannot prepare the bindings of a documentation build (DOCS_RS is set)."
                .to_string(),
            facts: vec![(
                "failed",
                format!(
                    "cannot copy {} to {}: {err}",
                    bindings.display(),
                    destination.display()
                ),
            )],
            advice: format!(
                "With DOCS_RS set, the bindings come from {DOCS_BINDINGS}, which ships inside\n\
                 this crate, instead of from a C bundle. Unset DOCS_RS to build against the C bundle.\n"
            ),
        }
    }
}

/// How to build without the download.
fn override_advice(build: &Build) -> String {
    let page = support::release_page_url(&build.version);
    let example = if cfg!(windows) {
        format!(
            "$env:VROBOTS_SDK_DIR = \"C:\\vrobots_sdk-cpp-{}\"; cargo build",
            build.version
        )
    } else {
        format!(
            "VROBOTS_SDK_DIR=/opt/vrobots_sdk-cpp-{} cargo build",
            build.version
        )
    };
    match support::bundle_asset(&build.version, &build.os, &build.arch, &build.env) {
        Some(asset) => format!(
            "To build without the download, fetch {} by hand from\n    {page}\n\
             unpack it into a folder of its own, and set VROBOTS_SDK_DIR to that folder, for example:\n    {example}\n",
            asset.file_name
        ),
        None => format!(
            "To build for {}, set VROBOTS_SDK_DIR to the folder of an unpacked C bundle {}\n\
             built for that target (bindings.rs at its root, the library in lib/), for example:\n    {example}\n\
             The published bundles are listed on\n    {page}\n",
            build.triple, build.version
        ),
    }
}

impl fmt::Display for Failure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "vrobots-sdk-sys: {}", self.summary)?;
        if !self.facts.is_empty() {
            writeln!(f)?;
            for (label, value) in &self.facts {
                writeln!(f, "  {:<11}{value}", format!("{label}:"))?;
            }
        }
        writeln!(f)?;
        write!(f, "{}", self.advice)
    }
}
