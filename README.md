# VRobots SDK

VRobots SDK is the client SDK for the Ubicoders virtual robots simulator. It
lets a program read a simulated robot's state, send it commands, receive camera
frames and call the simulator's services, with the same shape in every
language: construct a robot handle, connect, then run a plain control loop. One
core library does the work, and the Python, C++ and Rust front ends are thin
layers over it, so all three behave the same way and report the same numbers.

## Install

Every prebuilt piece of the SDK is **x86-64 only**: Windows x86-64 (MSVC) and
Linux x86-64. There is no ARM build (Apple Silicon, Raspberry Pi, AArch64
Linux, Windows on ARM) and no macOS build at present. Pick your language:


### Rust

```sh
cargo add vrobots-sdk
```

That is the whole install. The [`vrobots-sdk`](crates/vrobots-sdk) crate on
crates.io is a safe wrapper over the same C library, for Rust 1.88 or later
(install Rust with [rustup](https://rustup.rs) if you do not have it). Its
dependency `vrobots-sdk-sys` is fetched by cargo on its own; you never name it.
On the first build the crate downloads the C bundle of its own version from the
[Releases page](https://github.com/ubicoders/vrobots-sdk/releases) and checks it
against that Release's `SHA256SUMS`. Offline builds (`VROBOTS_SDK_DIR`) and the
`static` feature are in the [crate README](crates/vrobots-sdk/README.md).

### Python

```sh
pip install ubicoders-vrsdk
```

That is the whole install. The package is imported as `vrsdk` and supports
Python 3.8 and later. It also installs the `vrobots` command-line tool;
`vrobots topic list` shows what the simulator is publishing.

### C++

Clone this repository and run one script. It downloads the C bundle of this
checkout's version from the Releases page, verifies it against `SHA256SUMS`,
and unpacks it next to the repository, where the examples' CMake build finds it
on its own:

```sh
git clone https://github.com/ubicoders/vrobots-sdk
cd vrobots-sdk
bash scripts/get_cpp_bundle.sh          # Linux x86-64
# pwsh scripts/get_cpp_bundle.ps1       # Windows x86-64 (MSVC)
cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
cmake --build target/cpp-build --config Release
./target/cpp-build/ex01_hello_states    # Windows: .\target\cpp-build\Release\ex01_hello_states.exe
```

The unpacked folder holds `include/` (the C header `vrobots_sdk.h` and the
header-only C++17 wrapper `vrobots_sdk.hpp`) and `lib/` (the `vrobots_sdk_capi`
library). For a project of your own, add `include/` to the include path and
link `lib/libvrobots_sdk_capi.so` on Linux or the import library
`lib/vrobots_sdk_capi.dll.lib` on Windows, keeping `vrobots_sdk_capi.dll`
beside the executable. [`examples/cpp`](examples/cpp) shows a complete CMake
setup.

## Downloads

Every release on the
[Releases page](https://github.com/ubicoders/vrobots-sdk/releases) publishes
these assets:

| Asset | File name | Use |
|---|---|---|
| Python wheel, Windows x86_64 | `ubicoders_vrsdk-<version>-cp38-abi3-win_amd64.whl` | Python 3.8+ on Windows |
| Python wheel, Linux x86_64 | `ubicoders_vrsdk-<version>-cp38-abi3-manylinux_*_x86_64.whl` | Python 3.8+ on Linux |
| C bundle, Windows x86_64 | `vrobots_sdk-cpp-<version>-windows-x86_64.zip` | C, C++ and Rust on Windows (MSVC) |
| C bundle, Linux x86_64 | `vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz` | C, C++ and Rust on Linux (glibc 2.28 or newer) |
| Checksums | `SHA256SUMS` | SHA-256 checksums of every asset above |

`pip install ubicoders-vrsdk` fetches the matching wheel from PyPI, and a Rust
build fetches the matching C bundle on its own, so Python and Rust users need
these downloads only for offline installs. Verify a download with
`sha256sum -c SHA256SUMS --ignore-missing` on Linux.

## Book

The VRobots Book explains the concepts, the robots, the cameras and the
services, with every code sample in Python, C++ and Rust:
<https://ubicoders.github.io/vrobots-sdk/>. Its sources are in [`book/`](book).

## Examples

- [`examples/python`](examples/python): complete programs against the
  installed `ubicoders-vrsdk` package.
- [`examples/cpp`](examples/cpp): the same programs in C++, built with CMake
  against the downloaded C bundle.
- [`examples/rust`](examples/rust): the same programs in Rust, against the
  [`vrobots-sdk`](crates/vrobots-sdk) crate; run one from the repository root
  with `cargo run -p vrobots-examples --bin ex01_hello_states`.

Each example is a complete program: setup, then a plain loop. Start the
simulator in Play mode before running one.

## Licence

VRobots SDK is released under the
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International](LICENSE)
licence (CC BY-NC-SA 4.0). In plain terms:

- **Non-commercial.** You may use, copy and modify the SDK, the examples and the
  book for study, teaching, research and personal projects. Commercial use is
  not permitted. For a commercial licence, contact Ubicoders.
- **Share-alike.** If you distribute a modified version or a work built on the
  SDK, you must publish it under the same licence, with attribution.
- **No warranty.** The software is provided as is, without warranty or liability
  of any kind.
- **No patents (addendum).** [`LICENSE-ADDENDUM`](LICENSE-ADDENDUM) forbids patenting the SDK, anything it
  does, or any work derived from it. Any such patent is automatically licensed
  royalty-free to everyone, and filing one, or suing over one, ends your rights
  under the licence.

The full legal text is in [`LICENSE`](LICENSE) and [`LICENSE-ADDENDUM`](LICENSE-ADDENDUM).
