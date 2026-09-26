# Introduction

This book teaches you to drive Ubicoders virtual robots from code, and then serves as the reference you come back to.

## What the SDK is

The VRobots SDK is the client SDK for controlling Ubicoders virtual robots running in a Unity
simulator. It talks to the simulator over two transports, [zenoh](appendix-d-glossary.md)
for state, commands and services and [iceoryx2](appendix-d-glossary.md) for camera frames,
with FlatBuffers on the wire in both directions.

One Rust core is the single implementation, and every language reaches it through a thin
layer: the Python package is a binding over it, the C++ SDK is a header-only wrapper over
its C library, and the `vrobots-sdk` Rust crate is a safe wrapper over the same C library.
The surfaces therefore cannot drift: the same lifecycle, the same snapshots, the same
timestamps, and the same stable error codes in
[Appendix C](appendix-c-errors.md).

## What you need

| Requirement | Notes |
|---|---|
| Python 3.8 or newer | `pip install ubicoders-vrsdk` is the entire SDK install. The wheel carries the compiled Rust core and the `vrobots` command, so no toolchain, no `flatc` and no clone are involved. Windows and Linux x86-64. |
| The example programs | The wheel ships the library, not the examples. A plain `git clone` of this repository gets them; the Python ones import `vrsdk` and nothing else. |
| The Unity simulator, in Play mode | Required for anything that talks to a robot. |
| A C++17 compiler, for C++ | The SDK itself comes prebuilt in the C bundle for your OS, a download from the [Releases page](https://github.com/ubicoders/vrobots-sdk/releases) that holds the C header, the header-only C++ wrapper and the `vrobots_sdk_capi` library, so nothing of the SDK is compiled on your machine. The examples build with CMake 3.16 or newer. Windows x86-64 (MSVC) and Linux x86-64 with glibc 2.28 or newer. |
| Rust 1.88 or newer, for Rust | The `vrobots-sdk` crate links the same prebuilt `vrobots_sdk_capi` library, and its build downloads the C bundle of the crate's own version from the [Releases page](https://github.com/ubicoders/vrobots-sdk/releases), so the SDK's library is never compiled on your machine. `cargo add vrobots-sdk` adds it to a project (install Rust with [rustup](https://rustup.rs) if needed). Windows x86-64 (MSVC) and Linux x86-64 with glibc 2.28 or newer. |

[Installing the SDK and the simulator](ch01-getting-started/01-install.md) covers them in
order.

## The one idea to internalise first

This SDK is **STM32-shaped, not Arduino-shaped**. Your program does its setup and then owns a
plain loop: inside `main()` in Rust and C++, and at the top level of the script in Python.
There is no base class, no runner, no `setup()` and `update()` callbacks, and the SDK never
calls your code. If you have used the older Python client, this is the single largest
difference, and every page in the book assumes it.

The first example shows the shape: the whole of `main` in Rust and C++, and the whole script
in Python.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex01_hello_states.rs`:

```rust
fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        let s = robot.states(); // immutable latest snapshot, never torn
        let [x, y, z] = s.kin.lin_pos;
        println!("State t={:.3} pos=({x:.3},{y:.2},{z:.2})", s.elapsed);
        robot.rate(HZ); // drift-compensated pacing, Hz
    }
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex01_hello_states.cpp`:

```cpp
int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();  // header and library must be the same release
        vrsdk::set_log_callback(on_log);

        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();  // blocks until the first state snapshot arrives
        std::printf("connected to sys_id %u\n", robot.sys_id());

        // ===== loop =====
        for (;;) {
            const vrsdk::State s = robot.states();  // latest snapshot, never torn
            const double* p = s.kin().lin_pos;
            std::printf("State t=%.3f pos=(%.3f,%.2f,%.2f)\n", s.elapsed, p[0], p[1], p[2]);
            robot.rate(HZ);  // drift-compensated pacing, Hz
        }
    } catch (const vrsdk::Error& e) {
        // `code()` is the SDK's stable number -- the same one Python's
        // VrError.code and the CLI's `error [N]` report.
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex01_hello_states.py`:

```python
"""ex01 - read the robot's states in a loop."""

from vrsdk import RobotType, VirtualRobot

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

while True:
    s = mr.states  # immutable latest snapshot, never torn
    x, y, z = s.kin.lin_pos
    print(f"State t={s.elapsed:.3f} pos=({x:.3f},{y:.2f},{z:.2f})")
    mr.rate(50)  # drift-compensated pacing, Hz
```

{{#endtab }}
{{#endtabs }}

The program prints one line per iteration at the rate `rate` paces it to, until you stop it
with Ctrl-C. The Python script does not catch that interrupt, so it ends with a
`KeyboardInterrupt` traceback, which is expected rather than a fault.
[The shape of a program](ch02-concepts/04-program-shape.md) explains why the loop belongs
to you rather than to the SDK.

## How the book is organised

| Part | Chapters | What it gives you |
|---|---|---|
| Tutorial | [1](ch01-getting-started/00-intro.md), [2](ch02-concepts/00-intro.md) | A robot moving, then the model that explains why it moved. |
| The API in four slices | [3](ch03-reading-state/00-intro.md) read, [4](ch04-commands/00-intro.md) write, [5](ch05-cameras/00-intro.md) image, [6](ch06-services/00-intro.md) configure | One slice of the surface per chapter, in the order you meet them. |
| Reference | [7](ch07-robots/00-intro.md) | Per-robot pages: identity, physical model, commands, services, quirks. |
| Diagnostics | [8](ch08-tooling/00-intro.md) | The `vrobots` command, discovery, rates, logging, testing with the simulator closed. |
| Appendices | [A](appendix-a-topics.md), [B](appendix-b-commands.md), [C](appendix-c-errors.md), [D](appendix-d-glossary.md) | Lookup tables: topics, command ids, error codes, vocabulary. |

Chapters 3 to 6 are independent of each other. Read chapter 2 before any of them, because
they all lean on the five rules it sets out.

## Reading paths

| You want | Start at |
|---|---|
| A robot moving in ten minutes | [Chapter 1, Getting started](ch01-getting-started/00-intro.md) |
| To understand what you are doing | [Chapter 2, Concepts](ch02-concepts/00-intro.md) |
| Something is broken | [Chapter 8, Tooling and diagnostics](ch08-tooling/00-intro.md), then [When nothing happens](ch01-getting-started/08-troubleshooting.md) |

## Examples

Thirty-six complete programs live under `examples/rust/`, `examples/cpp/` and
`examples/python/`, the same programs under the same names in all three languages. Each is a
whole program rather than a snippet, with its settings written in the file, as constants at
the top or as literals at the call. The Python programs are short, flat scripts with no
`main()` function, and several leave out a side demonstration that their Rust and C++ twins
carry, such as a call made to be refused; the pages say so where it matters. The only
command-line argument any of them takes is a `sys_id`, because the ids of robots the scene
placed are handed out at load time and no constant can know them.

Every runnable page names its example once per language, in the order of the tabs:

```sh
cargo run -p vrobots-examples --bin ex01_hello_states
./target/cpp-build/ex01_hello_states
python examples/python/ex01_hello_states.py
```

The Python line runs from a clone of this repository once `pip install ubicoders-vrsdk` is
done. The C++ line assumes the CMake build in
[Installing the SDK and the simulator](ch01-getting-started/01-install.md); on Windows the
binary is `target\cpp-build\Release\ex01_hello_states.exe`. The Rust line runs from the root
of the same clone, where the Rust examples form the package `vrobots-examples`, and its first
build fetches the C library as that page describes.

Every Rust, C++ and Python block in this book is copied from one of those example files or
from a declaration the SDK ships, so what you read matches what you install. Rust
declarations come from the `vrobots-sdk` crate's source in `crates/vrobots-sdk/src/`, C++ ones
from the C bundle's `include/vrobots_sdk.h` and `include/vrobots_sdk.hpp`, and Python ones
from the type stubs that pip installs with the package, such as `vrsdk/_vrsdk.pyi`.

> **Note.** `cargo add vrobots-sdk` is the whole Rust install: the crate is on crates.io,
> and its build downloads the C bundle from the GitHub Release of the crate's own version.
> For an offline build, point `VROBOTS_SDK_DIR` at an unpacked bundle of the same version.

## Versions

This book documents SDK 0.1.11 against simulator v3.0.1. The IPC pins that release speaks are
flatbuffers 25.12.19, iceoryx2 0.9.3 and zenoh 1.9.0, and `vrobots --version` prints the set
your installed SDK actually carries. The pins are exact on purpose:
[Versions and pins](ch08-tooling/03-version-and-pins.md) explains what a caret pin one patch
off does, and why it looks like the simulator has stopped publishing.

## Licence

The SDK, the examples and this book are released under
[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/): free for study,
teaching, research and personal projects; not for commercial use; modified versions and
derived works must carry the same licence. `LICENSE-ADDENDUM` forbids patenting the SDK or any derived
work: such a patent is licensed royalty-free to everyone and ends the filer's rights. Both files
are in the repository root.

**Next:** [Getting started](ch01-getting-started/00-intro.md)

**See also:** [Five rules that explain everything](ch02-concepts/06-five-rules.md), [Appendix D: Glossary](appendix-d-glossary.md)
