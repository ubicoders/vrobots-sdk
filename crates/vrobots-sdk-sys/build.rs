// Build script for vrobots-sdk-sys.
//
// TODO: today this script does nothing. The planned behaviour is:
//
// 1. If the environment variable `VROBOTS_SDK_DIR` is set, treat it as the root
//    of an already unpacked C bundle (the folder holding `include/`, `lib/` and
//    `bindings.rs`) and skip the download. Emit
//    `cargo:rerun-if-env-changed=VROBOTS_SDK_DIR`.
// 2. Otherwise, download the C bundle for the target OS whose version equals
//    `CARGO_PKG_VERSION` from this repository's GitHub Release
//    (https://github.com/ubicoders/vrobots-sdk/releases/tag/v<version>):
//    `vrobots_sdk-cpp-<version>-windows-x86_64.zip` or
//    `vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz`.
// 3. Download `SHA256SUMS` from the same Release and verify the bundle against
//    it; fail the build on a mismatch or a missing entry.
// 4. Unpack the bundle into `OUT_DIR` and copy its `bindings.rs` to
//    `OUT_DIR/bindings.rs`, which `src/lib.rs` will `include!`.
// 5. Emit `cargo:rustc-link-search=native=<bundle>/lib` and
//    `cargo:rustc-link-lib=dylib=vrobots_sdk_capi`.

fn main() {}
