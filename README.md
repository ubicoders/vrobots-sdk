# VRobots SDK

VRobots SDK is the client SDK for the Ubicoders virtual robots simulator. It
lets a program read a simulated robot's state, send it commands, receive camera
frames and call the simulator's services, with the same shape in every
language: construct a robot handle, connect, then run a plain control loop. One
core library does the work, and the Python, C++ and Rust front ends are thin
layers over it, so all three behave the same way and report the same numbers.

## Install

### Python

```sh
pip install ubicoders-vrsdk
```

The package is imported as `vrsdk` and supports Python 3.8 and later. It also
installs the `vrobots` command-line tool; `vrobots topic list` shows what the
simulator is publishing.

### C++

1. Download the C bundle for your OS from the
   [Releases page](https://github.com/ubicoders/vrobots-sdk/releases) (see
   [Downloads](#downloads)).
2. Unpack it into a folder of its own. It contains `include/` (the C header
   `vrobots_sdk.h` and the header-only C++17 wrapper `vrobots_sdk.hpp`) and
   `lib/` (the `vrobots_sdk_capi` library).
3. Point CMake at `include/` and `lib/`:

   ```cmake
   target_include_directories(my_controller PRIVATE /path/to/bundle/include)
   target_link_libraries(my_controller PRIVATE /path/to/bundle/lib/<library file>)
   ```

   The library file is `libvrobots_sdk_capi.so` on Linux and the import library
   `vrobots_sdk_capi.dll.lib` on Windows, where `vrobots_sdk_capi.dll` must sit
   beside your executable. [`examples/cpp`](examples/cpp) shows a complete
   CMake setup.

### Rust (coming soon)

```sh
cargo add vrobots-sdk
```

The `vrobots-sdk` crate is not published yet. It is being developed in
[`crates/`](crates) as a safe wrapper over the same C library.

## Downloads

Every release on the
[Releases page](https://github.com/ubicoders/vrobots-sdk/releases) publishes
these assets:

| Asset | File name | Use |
|---|---|---|
| Python wheel, Windows x86_64 | `ubicoders_vrsdk-<version>-cp38-abi3-win_amd64.whl` | Python 3.8+ on Windows |
| Python wheel, Linux x86_64 | `ubicoders_vrsdk-<version>-cp38-abi3-manylinux_*_x86_64.whl` | Python 3.8+ on Linux |
| C bundle, Windows x86_64 | `vrobots_sdk-cpp-<version>-windows-x86_64.zip` | C and C++ on Windows (MSVC) |
| C bundle, Linux x86_64 | `vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz` | C and C++ on Linux (glibc 2.28 or newer) |
| Checksums | `SHA256SUMS` | SHA-256 checksums of every asset above |

`pip install ubicoders-vrsdk` fetches the matching wheel from PyPI, so the wheel
downloads are only needed for offline installs. Verify a download with
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
- [`examples/rust`](examples/rust): the Rust examples arrive with the
  `vrobots-sdk` crate.

Each example is a complete program: setup, then a plain loop. Start the
simulator in Play mode before running one.
