# Installing the SDK and the simulator

The SDK is installed per language: one command for Python, one download for C++, one
`cargo add` for Rust. Pick your section below, then get a simulator running on Windows,
Ubuntu or WSL.

> **Platforms.** Every prebuilt piece of the SDK is built for **x86-64 only**: Windows
> x86-64 (MSVC) and Linux x86-64. There is no ARM build (no Apple Silicon, no Raspberry Pi,
> no AArch64 Linux, no Windows on ARM) and no macOS build at present. On any other
> platform, `pip` stops with `No matching distribution found`, there is no C bundle to
> download, and the Rust crate's build stops with a message naming the supported targets.

| Language | Install | Runtime requirement |
|---|---|---|
| Python | `pip install ubicoders-vrsdk` | Python 3.8 or newer; Linux glibc 2.17 or newer |
| C++ | download the C bundle from the [Releases page](https://github.com/ubicoders/vrobots-sdk/releases) | a C++17 compiler; Linux glibc 2.28 or newer |
| Rust | `cargo add vrobots-sdk` | Rust 1.88 or newer; Linux glibc 2.28 or newer |

Every route uses the same prebuilt `vrobots_sdk_capi` core, so nothing of the SDK is ever
compiled on your machine, and the three languages behave identically.

## Python

```sh
pip install ubicoders-vrsdk
```

That command is the whole SDK install: the wheel carries the compiled Rust core, so no Rust
toolchain, no `flatc`, no `protoc` and no repository clone is involved. It puts two things on
your machine: `vrsdk`, the package every Python example in this book imports, and `vrobots`,
the command line tool of [The vrobots command](../ch08-tooling/01-cli.md), which runs the Rust
core's own command line code rather than a second implementation.

| Requirement | Version | Notes |
|---|---|---|
| Python | 3.8 or newer | one `abi3` wheel per platform covers 3.8 through 3.13 and later |
| Platform | Windows x86-64, Linux x86-64 | Linux needs glibc 2.17 or newer (`manylinux2014`); no ARM or macOS wheel |
| The Unity simulator | in Play mode | required by anything that talks to a robot |

`numpy` arrives with the wheel, because `frame.image` hands back an ndarray. `opencv-python`
does not, and three of the Python camera examples need it:

```sh
pip install "ubicoders-vrsdk[examples]"
```

The Python programs of [Hello image](06-hello-image.md),
[Saving a frame](../ch05-cameras/07-saving-frames.md) and
[Showing frames in a window](../ch05-cameras/08-showing-frames.md) import `cv2` at the top,
so without OpenCV they stop at that import. Among the C++ programs only the window one,
`ex34_camera_view`, needs OpenCV, and the C++ build skips it when OpenCV is not installed.

> **Gotcha.** No source distribution is published, deliberately: the core is built and
> released as prebuilt wheels only. On a platform with no wheel, pip therefore stops
> with `No matching distribution found for ubicoders-vrsdk` rather than starting a compile
> that cannot finish.

The wheel ships the library, not the example programs the pages of this book run. Those live
in the repository, and the Python ones need nothing from it but themselves:

```sh
git clone https://github.com/ubicoders/vrobots-sdk
python vrobots-sdk/examples/python/ex01_hello_states.py
```

Every Python example imports `vrsdk` and nothing else from the tree, so one file copied out
of it runs just as well on its own.

## C++

C++ needs no build of the SDK: download the C bundle for your OS from
<https://github.com/ubicoders/vrobots-sdk/releases> and unpack it into a folder of its own.
Two bundles exist, `vrobots_sdk-cpp-<version>-windows-x86_64.zip` (MSVC) and
`vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz` (glibc 2.28 or newer). The archive has no
top-level folder, and it holds everything a C++ program compiles and links against:

| Path | What it is |
|---|---|
| `include/vrobots_sdk.h` | The C API. |
| `include/vrobots_sdk.hpp` | The header-only C++17 wrapper. It includes `vrobots_sdk.h`, so keep the two in one folder. |
| `lib/` | The prebuilt `vrobots_sdk_capi` library: `libvrobots_sdk_capi.so` on Linux, `vrobots_sdk_capi.dll` and its import library `vrobots_sdk_capi.dll.lib` on Windows. |
| `examples/` | The C++ example programs, which also build inside the unpacked bundle. |
| `bindings.rs` | The same C API declared for Rust, used when the Rust crate builds. C++ ignores it. |
| `LICENSE` | The licence the SDK is released under. |

The Releases page also carries `SHA256SUMS`, the checksums of every asset, so on Linux
`sha256sum -c SHA256SUMS --ignore-missing` verifies a download.

To build the examples from your clone of this repository, follow
[`examples/cpp/README.md`](https://github.com/ubicoders/vrobots-sdk/blob/main/examples/cpp/README.md).
CMake picks up a bundle unpacked beside the repository on its own, or takes its location
from `-DVROBOTS_SDK_DIR`:

```sh
cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
cmake --build target/cpp-build --config Release
```

That puts one binary per example under `target/cpp-build/`, which is the path the `sh` block
on each page names. On Windows the binaries land in `target\cpp-build\Release\` with an
`.exe` suffix and the DLL copied beside each one; on Linux the build rpath points at the
bundle's `lib/`, so no `LD_LIBRARY_PATH` is needed.

## Rust

```sh
cargo add vrobots-sdk
```

That is the whole Rust install. If you do not have Rust yet, install it with
[rustup](https://rustup.rs), which puts `cargo` and the compiler on your machine together;
the crate needs Rust 1.88 or newer, which any current stable toolchain satisfies. The crate
is a safe wrapper over the same `vrobots_sdk_capi` library. Its dependency `vrobots-sdk-sys`
is fetched by cargo on its own; you never name it. On the first build the crate downloads the
C bundle of its own version from the Releases page, checks it against `SHA256SUMS` and links
it, so `cargo run` finds the library without any step of yours.

The Rust examples are the package `vrobots-examples` of this repository's workspace, and
they run from the root of your clone:

```sh
git clone https://github.com/ubicoders/vrobots-sdk
cd vrobots-sdk
cargo run -p vrobots-examples --bin ex01_hello_states
```

For an offline build, unpack the C bundle into a folder of its own, set `VROBOTS_SDK_DIR` to
the absolute path of that folder (the one holding `bindings.rs`, `include/` and `lib/`), and
put its `lib/` folder on `LD_LIBRARY_PATH` on Linux or `PATH` on Windows before running.
[`examples/rust/README.md`](https://github.com/ubicoders/vrobots-sdk/blob/main/examples/rust/README.md)
covers both routes, and the `static` feature, which links the static library so that a
program needs no shared library at run time.

## Getting the simulator

Prebuilt simulator packages are at <https://www.ubicoders.com/virtualrobots>. Windows 11
and Ubuntu 22.04 or newer are supported; macOS is not supported yet.

### Windows

Run `virtual_robots.exe`.

### Ubuntu

The build needs `xdg-utils` present and its own executable bit set.

```sh
sudo apt install xdg-utils -y
sudo chmod +x ./virtual_robots.x86_64
./virtual_robots.x86_64
```

Double-clicking `virtual_robots.x86_64` in a file manager works as well.

### WSL

WSL needs a graphics bridge before Unity will render. Install the Mesa and Vulkan packages
and force the D3D12 gallium driver, which routes rendering to the Windows GPU rather than
to the CPU rasteriser.

Save this as `install_wsl_graphics.bash`:

```bash
#!/bin/bash
# 1. Install necessary drivers and diagnostic tools
sudo apt-get update
sudo apt install xdg-utils -y
sudo apt install mesa-utils mesa-vulkan-drivers vulkan-tools -y

# 2. Add GPU bridge variables to .bashrc for persistence
# We use GALLIUM_DRIVER to force the D3D12 bridge (Windows GPU)
# and VK_ICD_FILENAMES to ensure Vulkan doesn't default to the CPU (llvmpipe)
if ! grep -q "GALLIUM_DRIVER=d3d12" ~/.bashrc; then
  echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
  echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc
fi

# 3. Reload environment
source ~/.bashrc

# 4. Verify the setup
echo "--- Checking OpenGL  ---"
glxinfo -B | grep -E "Device|Accelerated"

echo "--- Checking Vulkan  ---"
vulkaninfo | grep "Vulkan Instance Version"
vkcube
```

Run it, then reload your shell so the exported variables apply:

```sh
bash install_wsl_graphics.bash && source ~/.bashrc
```

Check that Vulkan came up on the GPU. `vkcube` should open a spinning cube window; if it
does not, the simulator will not render either.

```sh
vulkaninfo | grep "Vulkan Instance Version"
vkcube
```

Launch the simulator with Vulkan forced. The `LD_LIBRARY_PATH` edit removes
`/opt/zenoh-c/lib` from the loader path, so the simulator loads its own vendored zenoh
rather than a system copy.

```bash
#!/bin/bash
sudo chmod +x ./virtual_robots.x86_64
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | sed 's|:/opt/zenoh-c/lib||; s|/opt/zenoh-c/lib:||; s|/opt/zenoh-c/lib||')
nohup ./virtual_robots.x86_64 -force-vulkan > output.log 2>&1 &
```

> **Gotcha.** Camera frames ride iceoryx2 shared memory, so they are same-host only.
> A simulator running under WSL and a client running on Windows are two hosts as far as
> iceoryx2 is concerned: states arrive over zenoh, frames never do. Run both sides in the
> same place when you want images.

## Verifying the install

This prints what the SDK you just installed actually speaks, and it needs no simulator.

```sh
vrobots --version
```

```text
vrobots-sdk 0.1.11
  vrobots_msgs  v2.0.2-31-gac335c0 (schema_version 3)
  flatbuffers   25.12.19
  zenoh         1.9.0
  iceoryx2      0.9.3
  src_id        122
```

The first line is the SDK release. The second names the revision of the `vrobots_msgs`
message schemas the FlatBuffers code was generated from, with the schema version beside it,
so it moves when the schema does. The three pins are exact rather than caret ranges:
iceoryx2 compares major, minor and patch on every shared-memory open, and a version one
patch off does not error, it silently delivers nothing. That is the first thing to compare
against the simulator build when fields look like garbage or a camera stream never appears.

**Next:** [First contact: is anything publishing?](02-first-contact.md)

**See also:** [Versions and pins](../ch08-tooling/03-version-and-pins.md), [When nothing happens](08-troubleshooting.md)
