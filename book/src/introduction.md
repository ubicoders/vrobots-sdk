# Introduction

This book teaches you to drive Ubicoders virtual robots from code, and then serves as the reference you come back to.

## What the SDK is

`vrobots_sdk` is the Rust SDK for controlling Ubicoders virtual robots running in a Unity
simulator. It talks to the simulator over two transports, [zenoh](appendix-d-glossary.md)
for state, commands and services and [iceoryx2](appendix-d-glossary.md) for camera frames,
with FlatBuffers on the wire in both directions.

The Rust crate is the single implementation. The C++ and Python SDKs are thin bindings over
it, so the three surfaces cannot drift: the same lifecycle, the same snapshots, the same
timestamps, and the same stable error codes in
[Appendix C](appendix-c-errors.md).

## What you need

| Requirement | Notes |
|---|---|
| Python 3.8 or newer | `pip install ubicoders-vrsdk` is the entire SDK install. The wheel carries the compiled Rust core and the `vrobots` command, so no toolchain, no `flatc` and no clone are involved. Windows and Linux x86-64. |
| The example programs | The wheel ships the library, not the examples. A plain `git clone` of this repository gets them; the Python ones import `vrsdk` and nothing else. |
| The Unity simulator, in Play mode | Required for anything that talks to a robot. |
| Rust 1.89 or newer | Only to work in Rust or C++ from source. The crate is edition 2024, and the clone needs `--recurse-submodules` for the private `vrobots_msgs` submodule that ships the generated FlatBuffers code. |

[Installing the SDK and the simulator](ch01-getting-started/01-install.md) covers them in
order.

## The one idea to internalise first

This SDK is **STM32-shaped, not Arduino-shaped**. `main()` does setup and then owns a plain
loop. There is no base class, no runner, no `setup()` and `update()` callbacks, and the SDK
never calls your code. If you have used the older Python client, this is the single largest
difference, and every page in the book assumes it.

The whole of `main` in the first example shows the shape.


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
def main() -> None:
    # ===== setup =====
    vrsdk.init_logging("info")
    mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=SYS_ID)
    mr.connect()

    # ===== loop =====
    while True:
        s = mr.states  # immutable latest snapshot, never torn
        x, y, z = s.kin.lin_pos
        print(f"State t={s.elapsed:.3f} pos=({x:.3f},{y:.2f},{z:.2f})")
        mr.rate(HZ)  # drift-compensated pacing, Hz
```

{{#endtab }}
{{#endtabs }}

The program prints one line per iteration at the rate `robot.rate` paces it to, until you
stop it with Ctrl+C. [The shape of a program](ch02-concepts/04-program-shape.md) explains
why the loop belongs to you rather than to the SDK.

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

Thirty-three complete programs live under `examples/rust/src/bin/`. Each is a real `fn main`
rather than a snippet, takes no command-line arguments (settings are constants at the top of
the file), and is mirrored one for one in Python and C++ under `examples/python/` and
`examples/cpp/`, with the same numbers and the same behaviour.

Run one by its bin name, or by the equivalent name in the language you are using:

```sh
cargo run -p vrobots-examples --bin ex01_hello_states
./target/cpp-build/ex01_hello_states
python examples/python/ex01_hello_states.py
```

The C++ line assumes the build in
[Installing the SDK and the simulator](ch01-getting-started/01-install.md); on Windows the
binary is `target\cpp-build\Release\ex01_hello_states.exe`.

Every code block in this book is copied from one of those files or from a signature in the
SDK source, so anything you read here compiles as written.

## Versions

This book documents SDK 0.1.4 against simulator v3.0.0. The IPC pins that build speaks are
flatbuffers 25.12.19, iceoryx2 0.9.3 and zenoh 1.9.0, and `vrobots --version` prints the set
your build actually carries. The pins are exact on purpose:
[Versions and pins](ch08-tooling/03-version-and-pins.md) explains what a caret pin one patch
off does, and why it looks like the simulator has stopped publishing.

**Next:** [Getting started](ch01-getting-started/00-intro.md)

**See also:** [Five rules that explain everything](ch02-concepts/06-five-rules.md), [Appendix D: Glossary](appendix-d-glossary.md)
