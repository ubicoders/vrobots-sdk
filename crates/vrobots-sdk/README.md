# vrobots-sdk

The Rust client SDK for the Ubicoders virtual robots simulator: connect to a
simulated multirotor, truck, mass-spring-damper, cart-pole, half-drone or fixed
wing, read its state, send it commands, stream its cameras and configure it
through its services.

This crate is a safe wrapper over the VRobots SDK C library, whose raw
declarations are the `vrobots-sdk-sys` crate. The same library backs the C++
header and the Python package (`pip install ubicoders-vrsdk`), so the three
languages connect, time out and fail in exactly the same way.

## Install

```sh
cargo add vrobots-sdk
```

Supported targets are x86_64 Linux with glibc 2.28 or newer, and x86_64 Windows
with the MSVC toolchain. The minimum Rust version is 1.88.

## A first program

With the simulator in Play mode, this attaches to robot 1 of the test scene and
prints its position ten times a second:

```rust,no_run
use vrobots_sdk::{RobotType, VirtualRobot, VrError};

fn main() -> Result<(), VrError> {
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(1))?;
    loop {
        let s = robot.states();
        let [x, y, z] = s.kin.lin_pos;
        println!("t={:.3} pos=({x:.2},{y:.2},{z:.2})", s.elapsed);
        robot.rate(10.0);
    }
}
```

`Some(sys_id)` attaches to an existing robot; `None` creates a new one. Dropping
the `VirtualRobot` closes the session and leaves the robot running.

## Where the C library comes from

At build time the `vrobots-sdk-sys` build script downloads the C bundle of the
same version from the GitHub Release of
<https://github.com/ubicoders/vrobots-sdk>, verifies it against the Release's
`SHA256SUMS` and links the shared library `vrobots_sdk_capi`. `cargo run` and
`cargo test` then find the library on their own.

- **Offline builds**: set `VROBOTS_SDK_DIR` to the folder of an unpacked C
  bundle of the same version, and add its `lib/` folder to `LD_LIBRARY_PATH`
  (Linux) or `PATH` (Windows) to run.
- **No shared library at run time**: enable the `static` feature to link the
  static library instead.
- **Shipping a program**: copy `libvrobots_sdk_capi.so` (Linux) or
  `vrobots_sdk_capi.dll` (Windows) next to the executable, or use `static`.

The `vrobots-sdk-sys` documentation describes these options in full.

## What is in the crate

| Module | What it holds |
|---|---|
| `robot` | `VirtualRobot`: connect, read state, pace a loop, delete |
| `options` | `RobotType` and `ConnectOptions` |
| `state` | `State` and its blocks, the `Axes` and `EulerOrder` wire tags |
| `commands` | the command methods, the `cmd` ids and `CmdArgs` |
| `setpoint` | reading other clients' commands: `SetpointStream` |
| `camera` | `CameraStream`, `Frame` and the camera options |
| `services` | reset, activate and the configuration requests |
| `topics` | every wire name, built in one place |
| `discovery` | `list_topics` and `measure_rate` |
| `rotations` | quaternions, matrices, Euler angles and frame conversions |
| `version` | `version_info` and `check_version` |
| `logging` | `init_logging`, `set_log_callback` and `set_log_level` |
| `error` | `VrError`, one variant per error code of the C library |

The book at <https://ubicoders.github.io/vrobots-sdk/> explains the simulator,
the wire and every call, with each sample in Rust, C++ and Python.

## Licence

The VRobots SDK, including this crate and the C library it links, is released
under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
International licence (CC BY-NC-SA 4.0) with a patent addendum. The full texts
are included in this crate as `LICENSE` and `LICENSE-ADDENDUM`, and are also at
the root of <https://github.com/ubicoders/vrobots-sdk>.
