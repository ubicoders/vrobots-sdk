# Installing the SDK and the simulator

Install the SDK with one `pip` command, then get a simulator running on Windows, Ubuntu or WSL.

```sh
pip install ubicoders-vrsdk
```

## What pip gives you

That command is the whole SDK install: the wheel carries the compiled Rust core, so no Rust
toolchain, no `flatc`, no `protoc` and no repository clone is involved. It puts two things on
your machine: `vrsdk`, the package every Python example in this book imports, and `vrobots`,
the command line tool of [The vrobots command](../ch08-tooling/01-cli.md), which is the same
program as the Rust build's rather than a second implementation.

| Requirement | Version | Notes |
|---|---|---|
| Python | 3.8 or newer | one `abi3` wheel per platform covers 3.8 through 3.13 and later |
| Platform | Windows x86-64, Linux x86-64 | Linux needs glibc 2.17 or newer (`manylinux2014`); macOS is not published yet |
| The Unity simulator | in Play mode | required by anything that talks to a robot |

`numpy` arrives with the wheel, because `frame.image` hands back an ndarray. `opencv-python`
does not, and it is what the camera examples want in order to open a window:

```sh
pip install "ubicoders-vrsdk[examples]"
```

Without it, [Hello image](06-hello-image.md) prints metadata instead of opening a window and
[Saving a frame](../ch05-cameras/07-saving-frames.md) writes a PPM instead of a PNG. The one
page that needs OpenCV outright is [Showing frames in a window](../ch05-cameras/08-showing-frames.md),
in every language.

> **Gotcha.** No source distribution is published, deliberately: the core is built and
> released as prebuilt wheels only. On a platform with no wheel, pip therefore stops
> with `No matching distribution found for ubicoders-vrsdk` rather than starting a compile
> that cannot finish.

## Getting the example programs

The wheel ships the library, not the example programs the pages of this book run. Those live
in the repository, and the Python ones need nothing from it but themselves:

```sh
git clone https://github.com/ubicoders/vrobots-sdk
python vrobots-sdk/examples/python/ex01_hello_states.py
```

Every Python example imports `vrsdk` and nothing else from the tree, so one file copied out
of it runs just as well on its own.

## The C++ and Rust surfaces

Skip this section unless you are working in C++ or Rust. C++ needs no build of the SDK:
download the C bundle for your OS from <https://github.com/ubicoders/vrobots-sdk/releases>,
unpack it into a folder of its own, and follow
[`examples/cpp/README.md`](https://github.com/ubicoders/vrobots-sdk/blob/main/examples/cpp/README.md)
in your clone. The bundle holds the C header, the header-only C++ wrapper and the
`vrobots_sdk_capi` library. CMake picks up a bundle unpacked beside the repository on its
own, or takes its location from `-DVROBOTS_SDK_DIR`:

```sh
cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
cmake --build target/cpp-build --config Release
```

That puts one binary per example under `target/cpp-build/`, which is the path the `sh` block
on each page names. On Windows the binaries land in `target\cpp-build\Release\` with an
`.exe` suffix and the DLL copied beside each one; on Linux the build rpath points at the
bundle's `lib/`, so no `LD_LIBRARY_PATH` is needed.

For Rust, the `vrobots-sdk` crate on crates.io is coming but not published yet. Until it is,
the Rust tabs in this book show the shape of the API, and there is nothing to install.

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
vrobots-sdk 0.1.4
  vrobots_msgs  v2.0.2-31-gac335c0 (schema_version 3)
  flatbuffers   25.12.19
  zenoh         1.9.0
  iceoryx2      0.9.3
  src_id        122
```

The first line is the SDK release. The second is a `git describe` of the `vrobots_msgs`
submodule the FlatBuffers code was generated from, so it moves when the schema does. The
three pins are exact rather than caret ranges: iceoryx2 compares major, minor and patch on
every shared-memory open, and a version one patch off does not error, it silently delivers
nothing. That is the first thing to compare against the simulator build when fields look
like garbage or a camera stream never appears.

**Next:** [First contact: is anything publishing?](02-first-contact.md)

**See also:** [Versions and pins](../ch08-tooling/03-version-and-pins.md), [When nothing happens](08-troubleshooting.md)
