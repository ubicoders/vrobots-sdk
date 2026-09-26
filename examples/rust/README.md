# Rust examples

Thirty-six complete programs, in book order, against the `vrobots-sdk` crate in
[`crates/vrobots-sdk`](../../crates/vrobots-sdk), the Rust wrapper over the
SDK's C library. They form the package `vrobots-examples`, one binary per
program. Each has its own `fn main` in the **STM32 shape**: `main()` does the
setup, then owns a plain loop. No base class, no runner, no update callback: the
loop is yours, and the SDK never calls into it.

**No command-line options anywhere.** Every setting is a `const` at the top of
the file or a literal at the call, and a permutation worth showing is its own
file rather than a flag. That is why ex14, ex15, ex16 and ex17 exist instead of
`--save`, `--resolution`, `--camera` and `--pose` on ex03.

The one exception is the **`sys_id` of a scene-authored robot**. Those robots
are not in any spawn catalog, and their ids are handed out at scene load and
keep incrementing, so no file can hard-code one. It is a single positional
argument after `--`: **required** by ex29 to ex33 and ex35, and **optional** for
ex21, ex22 and ex27, which create their own multirotor when it is left off:

```bash
cargo run -p vrobots-examples --bin ex31_globalhawk_direct -- 15
```

| Example | What it teaches |
|---|---|
| `ex01_hello_states` | read the always-fresh snapshot at your own rate |
| `ex02_hello_control` | `SET_MR_PWM` in a loop; the actuator echo is the only receipt |
| `ex03_hello_image` | camera frames alongside states: freshness, orientation, RGBA vs BGR |
| `ex04_hello_service` | create a robot and delete it: what "explicit lifecycle" means |
| `ex05_hello_car` | drive the truck with `SET_CAR` |
| `ex06_hello_throttle` | `set_mr_throttle`: what a command the sim ignores looks like |
| `ex07_body_wrench` | `set_body_force` / `_torque` / `_ft`, and frame-tagged vectors |
| `ex08_generic_cmd` | `send_cmd` + `CmdArgs`: the raw escape hatch, landed vs ignored |
| `ex09_state_paced_loop` | `wait_new_state`: one iteration per sample; a timeout is a status |
| `ex10_sensors_tour` | the whole `State`: truth vs measured vs believed |
| `ex11_topic_discovery` | `list_topics` from code; `observed` vs registered |
| `ex12_version_info` | version pins (including crate vs library), subscriber stats, `last_error` |
| `ex13_open_camera` | attaching to `front_left`, and what a wrong triple looks like |
| `ex14_camera_save` | one frame to disk as a PPM, then exit |
| `ex15_camera_formats` | what a format and a resolution cost, and the robot-wide rule |
| `ex16_two_cameras` | `front_left` + `front_right`, independent freshness, `t_ns` skew |
| `ex17_camera_pose` | the one example that mounts: `mount_camera_with`, pose, lens, and reading them back |
| `ex18_multi_robot` | two handles in one process, and their frames differ |
| `ex19_robust_loop` | survive the simulator stopping and restarting |
| `ex20_logging_tour` | diagnostics: returned errors vs the SDK's log events |
| `ex21_reset` | `reset()`: a bare GET, and why "home" is not "where you found it" |
| `ex22_physical_params` | change the mass mid-flight; the climb rate is the only receipt |
| `ex23_skins` | `set_skin`: the one service that ever answers "no" |
| `ex24_sensor_config` | noise models you can *see*, and the block that kills a sensor |
| `ex25_frames` | `set_frames` / `scene_frame()`: three levels, most specific wins |
| `ex26_drive_config` | retune the truck's drivetrain and measure the turn radius |
| `ex27_rotor_config` | replace the rotor list: a wrong length is dropped whole |
| `ex28_hello_msd` | a mass-spring-damper: the one plant you can predict |
| `ex29_hello_cartpole` | balance it, and measure the rail centre the wire omits |
| `ex30_hello_halfdrone` | two rotors, and `NoResponder` as the capability probe |
| `ex31_globalhawk_direct` | take the fixed wing's six panels off the autopilot |
| `ex32_fw_rate_controller` | your rate loop, the operator's stick, read off `z/cmd` |
| `ex33_fw_est_source` | feed the onboard loop an estimate instead of the truth |
| `ex34_camera_view` | `front_left` in an OpenCV window: RGBA to BGR, one `imshow` per frame; needs `--features opencv` |
| `ex35_publish_estimate` | `publish_estimate`: the onboard loop flies your attitude, and your 5 deg lie with it |
| `ex36_rotations` | `vrobots_sdk::rotations`: convert a live truck's state between frames, polar vs axial |

## Running them

From the root of a clone of this repository, with the simulator in Play mode:

```bash
cargo run -p vrobots-examples --bin ex01_hello_states
cargo run -p vrobots-examples --bin ex10_sensors_tour
cargo run -p vrobots-examples --bin ex31_globalhawk_direct -- 15
RUST_LOG=vrobots_sdk=debug cargo run -p vrobots-examples --bin ex20_logging_tour
```

Ctrl-C stops any of them; only ex17 should be left to finish (see below).
`cargo build -p vrobots-examples` builds every program except ex34, which needs
OpenCV. Check that the simulator is publishing first:

```bash
cargo run -p vrobots-examples --bin ex11_topic_discovery
```

`vrobots topic list`, the command-line tool that the Python package
(`pip install ubicoders-vrsdk`) installs, prints the same list.

The examples need Rust 1.88 or newer, on x86_64 Linux with glibc 2.28 or newer
or on x86_64 Windows with the MSVC toolchain.

## Where the C library comes from

The `vrobots-sdk` crate links `vrobots_sdk_capi`, the SDK's prebuilt C library,
through the `vrobots-sdk-sys` crate. Nothing of the SDK itself is compiled from
source. The library arrives in one of two ways.

**Downloaded by the build, the default.** The `vrobots-sdk-sys` build script
downloads the C bundle of its own version from the
[GitHub Release](https://github.com/ubicoders/vrobots-sdk/releases) of that
version, checks it against the Release's `SHA256SUMS` and links it. Later builds
reuse the verified copy without network access, and `cargo run` finds the shared
library on its own. This requires a Release for the crate's version: a checkout
of `main` can carry a version that has not been released yet, and then only
`VROBOTS_SDK_DIR` works.

**An unpacked bundle, named by `VROBOTS_SDK_DIR`.** For offline builds, for
networks that re-sign TLS traffic, and for a version that has no Release yet.
Download the C bundle for your OS from the Releases page (the same archive the
C++ examples build against), unpack it into a folder of its own, and set
`VROBOTS_SDK_DIR` to that folder, the one holding `bindings.rs`, `include/` and
`lib/`. Give an absolute path, because a relative one is resolved against the
`vrobots-sdk-sys` crate's own directory. The library then lives outside Cargo's
build directory, so a program starts only once the bundle's `lib/` folder is on
the loader path. With the bundle unpacked into `vrobots_sdk-cpp` beside the
clone, from the repository root:

```bash
# Linux
export VROBOTS_SDK_DIR="$PWD/../vrobots_sdk-cpp"
export LD_LIBRARY_PATH="$VROBOTS_SDK_DIR/lib"
cargo run -p vrobots-examples --bin ex01_hello_states
```

```powershell
# Windows
$env:VROBOTS_SDK_DIR = "$PWD\..\vrobots_sdk-cpp"
$env:PATH = "$env:VROBOTS_SDK_DIR\lib;$env:PATH"
cargo run -p vrobots-examples --bin ex01_hello_states
```

Without the loader path, a Linux program stops at start-up with
`error while loading shared libraries: libvrobots_sdk_capi.so`.

**No shared library at run time: `--features static`.** This links the static
library instead, so the program needs neither `LD_LIBRARY_PATH` nor `PATH`, and
a binary under `target/` runs on its own. It works with either source of the
bundle; it is verified on Linux and not yet tested on Windows.

```bash
cargo run -p vrobots-examples --features static --bin ex01_hello_states
```

The bundle and the crate must be the same release, because they share their
structs by layout. The build script warns when `VROBOTS_SDK_DIR` names a bundle
of another version, and `ex12_version_info` checks the pair again at run time
with `check_version()`. The `vrobots-sdk-sys` crate's
[README](../../crates/vrobots-sdk-sys/README.md) describes the build script, its
environment variables and the download mirror option in full.

## ex34 and OpenCV

`ex34_camera_view` is the one example that needs a library outside the SDK. It
sits behind the `opencv` feature, so that nothing else ever needs OpenCV:

```bash
cargo run -p vrobots-examples --features opencv --bin ex34_camera_view
```

The `opencv` crate binds to the OpenCV installed on the machine while it builds,
so building ex34 needs OpenCV 4 with its development files, plus `libclang`.
The `opencv` crate's documentation lists the packages for each OS. If your
system OpenCV needs a newer binding release, raise the `opencv` version in this
folder's `Cargo.toml`.

## System ids in the test scene

Sys ids are **allocated at scene load and keep incrementing across scene
loads**. On a fresh boot straight into the Flatworld scene the truck is `0` and
the multirotor `1`, but treat that as a convenience, not a contract. Every
example up to ex20 that attaches to a robot names its id in a `const`, with a
comment naming the robot; `ex11_topic_discovery` shows what is really there.

ex04, ex23 to ex26, ex28 and ex36 do not use those ids at all: they **create**
their own robot and delete it on the way out, so they leave the scene as they
found it. ex29 to ex33 and ex35 do the opposite: their robots are
scene-authored, so they attach to the `sys_id` you pass and never delete
anything. The Global Hawk of ex31 to ex33 and ex35 lives in the IMU scene, not
the sandbox. ex21, ex22 and ex27 do either, depending on whether you pass an id.
When you do, **ex22 and ex27 leave their configuration on that robot until the
scene is reloaded**, since neither mass nor rotor geometry can be read back to
restore. Pass one anyway for now: as of simulator v3.0.0 a client-created
multirotor does not move (its rigidbody never integrates, while its actuator
echo answers normally), so the create path shows no climb at all. Created
trucks and mass-spring-dampers have live physics, and the scene's own
multirotor flies.

## Five things that bite people

- **An ack is a receipt, not a result.** Every `srv/*` reply is packed the
  instant the request arrives, and the change lands in phase 0 of the robot's
  *next* physics step. Worse, only `srv/skin` ever answers `ok = false`: a wrong
  rotor count, an unknown frame id or a `drive_mode` that is not 2 or 4 is acked
  `ok` and refused by a log line inside the simulator that no client can see.
  The SDK refuses what it can before sending; for the rest, **the state stream
  is the confirmation**, which is why ex22, ex26 and ex27 measure instead of
  asserting.
- **Every vrobot already has `front_left` and `front_right`** (720p **rgba8**),
  so the camera examples `open_camera` one of them rather than creating their
  own. Opening mutates nothing, so those examples have no cleanup step and
  cannot collide with each other or with anything else reading the same stream.
  `mount_camera`, the add-a-camera API with its mount pose, lens and matching
  `unmount_camera`, appears in exactly one file, `ex17_camera_pose`. That is also
  the only one that must run to completion rather than being stopped with
  Ctrl-C, and the only one that can move the **resolution** knob, which is
  robot-wide: mounting at 360p restreams every camera on that robot under a new
  name (`front_left/360p_rgba8`), and unmounting does not put it back.
- **The robots do not agree about axes.** The truck publishes `"fru"` while the
  multirotor and the Global Hawk publish `"frd"` (all three verified live), so
  the third component is up for one and down for the others. Read
  `coord_frame_id`; `ex18_multi_robot` shows what happens when you do not, and
  `ex25_frames` is the service that decides it.
- **Frames are RGBA, not BGR.** Rows are already top-down and the stride is
  tight, but channel order is the renderer's, and the scene's cameras publish
  Unity's native four-channel readback. `frame.data` dereferences to a `&[u8]`
  of those bytes. Convert once, at the call site that wants BGR, as ex34 does
  with `COLOR_RGBA2BGR`.
- **`env.agl` is a hard-coded zero** in simulator v3.0.0, for every robot.
  Filling it needs a downward raycast the simulator does not run yet, and it
  publishes 0 rather than guessing: `env` is the truth block, and an invented
  height above ground is worse than an obviously absent one. Take altitude from
  `kin.lin_pos[2]` instead (negated on a robot publishing `"frd"`, where the
  third component is down), which is what ex22 and ex27 do.

## The twins

[`examples/python`](../python) and [`examples/cpp`](../cpp) carry the same
programs under the same names against the same SDK, and they print the same
numbers, which is the point: each language adds its own idiom, not its own
behaviour. Three differences are deliberate:

- `ex20_logging_tour` uses each language's own logging idiom: a `RUST_LOG`-style
  filter over the SDK's events here, the `logging` bridge in Python, a C
  callback in C++.
- `ex12_version_info` checks that the crate and the C library it links are one
  release, as its C++ twin checks the header against the library. A Python
  program has no such pair to check.
- `ex14_camera_save` writes a PPM, as in C++, where the Python twin writes a PNG
  through OpenCV.

## Licence

These examples are part of the VRobots SDK and are released under the Creative
Commons Attribution-NonCommercial-ShareAlike 4.0 International licence
(CC BY-NC-SA 4.0) with a patent addendum. The full text is in
[`LICENSE`](../../LICENSE) and [`LICENSE-ADDENDUM`](../../LICENSE-ADDENDUM) at
the root of this repository.
