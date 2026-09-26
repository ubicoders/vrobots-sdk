# Installing the SDK and the simulator

Three steps: check your platform, install the SDK for your language, get a simulator
running. Each is short.

## Platforms

| Platform | Python wheel | C bundle (C++) | Rust crate |
|---|---|---|---|
| Windows x86-64 | yes | yes (MSVC) | yes |
| Linux x86-64 | yes (glibc 2.17 or newer) | yes (glibc 2.28 or newer) | yes (glibc 2.28 or newer) |
| macOS | no | no | no |
| ARM, any OS (Apple Silicon, Raspberry Pi, AArch64 Linux, Windows on ARM) | no | no | no |

Every prebuilt piece of the SDK is built for x86-64 only. On an unsupported platform, `pip`
stops with `No matching distribution found`, there is no C bundle to download, and the Rust
crate's build stops with a message naming the supported targets.

## Install the SDK

Every route uses the same prebuilt `vrobots_sdk_capi` core, so nothing of the SDK is ever
compiled on your machine, and the three languages behave identically. The Unity simulator,
in Play mode, is required by anything that talks to a robot.

{{#tabs global="lang" }}
{{#tab name="Rust" }}

```sh
cargo add vrobots-sdk
```

That is the whole install. Needs Rust 1.88 or newer; if you do not have Rust, install it
with [rustup](https://rustup.rs). The crate's dependency `vrobots-sdk-sys` is fetched by
cargo on its own; you never name it. On the first build the crate downloads the C bundle
of its own version from the Releases page, checks it against `SHA256SUMS` and links it, so
`cargo run` finds the library without any step of yours. Offline builds and the `static`
feature are described in
[`examples/rust/README.md`](https://github.com/ubicoders/vrobots-sdk/blob/main/examples/rust/README.md).

{{#endtab }}
{{#tab name="C++" }}

Download the C bundle for your OS from the
[Releases page](https://github.com/ubicoders/vrobots-sdk/releases) and unpack it into a
folder of its own:

| File | For |
|---|---|
| `vrobots_sdk-cpp-<version>-windows-x86_64.zip` | Windows, MSVC |
| `vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz` | Linux, glibc 2.28 or newer |

The bundle holds `include/vrobots_sdk.h` (the C API), `include/vrobots_sdk.hpp` (the
header-only C++17 wrapper), `lib/` (the prebuilt library), the C++ examples and the
licence. Point your build at `include/` and `lib/`; a complete CMake setup is in
[`examples/cpp/README.md`](https://github.com/ubicoders/vrobots-sdk/blob/main/examples/cpp/README.md).
`SHA256SUMS` on the Releases page verifies a download
(`sha256sum -c SHA256SUMS --ignore-missing` on Linux).

{{#endtab }}
{{#tab name="Python" }}

```sh
pip install ubicoders-vrsdk
```

That is the whole install. Needs Python 3.8 or newer; one `abi3` wheel per platform covers
3.8 through 3.13 and later. It puts two things on your machine: `vrsdk`, the package every
Python example in this book imports, and `vrobots`, the command line tool of
[The vrobots command](../ch08-tooling/01-cli.md). `numpy` comes with it; the three camera
examples that open a window also need OpenCV:

```sh
pip install "ubicoders-vrsdk[examples]"
```

No source distribution is published, deliberately, so on a platform with no wheel pip stops
instead of starting a compile that cannot finish.

{{#endtab }}
{{#endtabs }}

## Get the examples

The packages ship the library, not the example programs the pages of this book run. Those
live in the repository, one program per language under the same name:

```sh
git clone https://github.com/ubicoders/vrobots-sdk
cd vrobots-sdk
```

{{#tabs global="lang" }}
{{#tab name="Rust" }}

```sh
cargo run -p vrobots-examples --bin ex01_hello_states
```

{{#endtab }}
{{#tab name="C++" }}

```sh
cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
cmake --build target/cpp-build --config Release
./target/cpp-build/ex01_hello_states
```

CMake finds a bundle unpacked beside the repository on its own, or takes `-DVROBOTS_SDK_DIR`.
On Windows the binaries land in `target\cpp-build\Release\` with the DLL beside them.

{{#endtab }}
{{#tab name="Python" }}

```sh
python examples/python/ex01_hello_states.py
```

Every Python example imports `vrsdk` and nothing else from the tree, so one file copied out
of it runs on its own.

{{#endtab }}
{{#endtabs }}

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
