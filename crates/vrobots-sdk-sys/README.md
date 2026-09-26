# vrobots-sdk-sys

Raw FFI bindings to `vrobots_sdk_capi`, the C library of the VRobots SDK, the
client SDK for the Ubicoders virtual robots simulator. The crate declares the C
API exactly as the header `vrobots_sdk.h` declares it; every function is
`unsafe`. Most programs should use the safe wrapper crate `vrobots-sdk` instead.

## How the C library is obtained

At build time the build script downloads the C bundle whose version equals the
crate version from the GitHub Release `v<version>` of
[ubicoders/vrobots-sdk](https://github.com/ubicoders/vrobots-sdk/releases),
checks it against the Release's `SHA256SUMS`, and unpacks it into the build
directory. Later builds reuse the verified copy without network access.

| Target | Asset |
|---|---|
| x86_64 Linux, glibc 2.28 or newer | `vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz` |
| x86_64 Windows, MSVC | `vrobots_sdk-cpp-<version>-windows-x86_64.zip` |

The download uses HTTPS through rustls (no OpenSSL) and honours `HTTPS_PROXY`,
`HTTP_PROXY`, `ALL_PROXY` and `NO_PROXY`.

## Environment variables

| Variable | Effect |
|---|---|
| `VROBOTS_SDK_DIR` | Use this unpacked C bundle instead of downloading one. It must hold `bindings.rs` at its root and the library in `lib/`, and be the bundle of the crate's version. Needed offline, on networks that re-sign TLS traffic, and for targets without a published bundle. |
| `VROBOTS_SDK_DOWNLOAD_URL` | Replaces the URL prefix `https://github.com/ubicoders/vrobots-sdk/releases/download/v<version>`, for a mirror that serves the same bundle and `SHA256SUMS`. |
| `DOCS_RS` | Set and not empty (docs.rs sets it): a documentation build. Nothing is downloaded or linked, `VROBOTS_SDK_DIR` is ignored, and the bindings come from a copy shipped in the crate. See "Documentation builds" below. |

## Features

| Feature | Effect |
|---|---|
| `static` | Link the static library instead of the shared one, so the program needs no shared library at run time. Verified on Linux; not yet tested on Windows. |

## Running programs

With the default shared library, the program must find it when it starts:
`libvrobots_sdk_capi.so` on the rpath or `LD_LIBRARY_PATH` on Linux,
`vrobots_sdk_capi.dll` beside the executable or on `PATH` on Windows.
`cargo run` and `cargo test` find a downloaded bundle on their own. With
`VROBOTS_SDK_DIR`, add the bundle's `lib/` folder to `LD_LIBRARY_PATH` or
`PATH`. Build scripts of crates that depend on this one directly receive the
folder as `DEP_VROBOTS_SDK_CAPI_LIB_DIR`.

Requires Rust 1.88 or newer.

## Documentation builds

docs.rs builds without network access. When `DOCS_RS` is set and not empty, as
on docs.rs, the build script downloads nothing, ignores `VROBOTS_SDK_DIR` and
links nothing: it compiles `bindings/vrobots_sdk.rs`, a copy of the bundle's
`bindings.rs` that ships in the crate for documentation builds only. In this
mode `cargo doc` and `cargo check` work, for this crate and for its dependents,
but anything that calls the library fails to link, so leave `DOCS_RS` unset
for ordinary builds. Every other build compiles the bundle's own
`bindings.rs`, and the crate's tests fail if the shipped copy differs from it.

## Licence

Released under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
International licence (CC BY-NC-SA 4.0) with a patent addendum. The full texts
are included in this crate as `LICENSE` and `LICENSE-ADDENDUM`, and are also in
the repository:
[`LICENSE`](https://github.com/ubicoders/vrobots-sdk/blob/main/LICENSE) and
[`LICENSE-ADDENDUM`](https://github.com/ubicoders/vrobots-sdk/blob/main/LICENSE-ADDENDUM).
